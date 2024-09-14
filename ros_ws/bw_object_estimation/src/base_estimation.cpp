#include "base_estimation.h"

BaseEstimation::BaseEstimation(ros::NodeHandle *nodehandle) : nh(*nodehandle), _tf_listener(_tf_buffer)
{
    _shared_config = new bw_shared_config::SharedConfig();
    *_shared_config = bw_shared_config::getSharedConfig();
    _info_sub = nh.subscribe<sensor_msgs::CameraInfo>("camera_info", 1, &BaseEstimation::camera_info_callback, this);
    _field_sub = nh.subscribe<bw_interfaces::EstimatedObject>("field", 1, &BaseEstimation::field_callback, this);
}

BaseEstimation::~BaseEstimation()
{
}

void BaseEstimation::camera_info_callback(const sensor_msgs::CameraInfoConstPtr &camera_info)
{
    _camera_model.fromCameraInfo(camera_info);
    _info_sub.shutdown();
    ROS_INFO("Camera model loaded");
}

void BaseEstimation::field_callback(const bw_interfaces::EstimatedObjectConstPtr &field)
{
    ROS_INFO("Field received");
    _field = *field;
    _plane_computed = false;
    field_received_callback();
}

bool BaseEstimation::compute_plane()
{
    sensor_msgs::CameraInfo info = _camera_model.cameraInfo();
    if (info.header.frame_id.empty())
    {
        ROS_ERROR("Camera info frame_id is empty. Cannot transform field to camera frame.");
        return false;
    }
    geometry_msgs::TransformStamped transform;
    try
    {
        transform = _tf_buffer.lookupTransform(info.header.frame_id, _field.header.frame_id, ros::Time(0), ros::Duration(5.0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("Failed to look up transform from %s to %s. %s", info.header.frame_id.c_str(), _field.header.frame_id.c_str(), ex.what());
        return false;
    }

    // Extract the position and orientation from the pose
    auto position = transform.transform.translation;
    auto orientation = transform.transform.rotation;

    // Convert the position and orientation to cv::Mat
    _plane_center = cv::Point3d(position.x, position.y, position.z);

    // Convert the orientation to a quaternion
    Eigen::Quaterniond quat(orientation.w, orientation.x, orientation.y, orientation.z);
    Eigen::Matrix3d rotation_matrix = quat.normalized().toRotationMatrix();
    Eigen::Vector3d normal = rotation_matrix * Eigen::Vector3d(0, 0, 1);
    normal = normal.normalized();
    _plane_normal = cv::Point3d(normal.x(), normal.y(), normal.z());

    return true;
}

cv::Point3d BaseEstimation::get_plane_center()
{
    if (!_plane_computed && compute_plane())
    {
        _plane_computed = true;
    }
    return _plane_center;
}
cv::Point3d BaseEstimation::get_plane_normal()
{
    if (!_plane_computed && compute_plane())
    {
        _plane_computed = true;
    }
    return _plane_normal;
}

bool BaseEstimation::project_to_field(
    cv::Point2d centroid_uv,
    cv::Point3d plane_center,
    cv::Point3d plane_normal,
    cv::Point3d &out_point,
    double epsilon)
{
    cv::Point3d root_vector = _camera_model.projectPixelTo3dRay(centroid_uv);
    cv::Point3d origin = cv::Point3d(0, 0, 0);
    double dot = plane_normal.dot(root_vector);
    if (std::abs(dot) > epsilon)
    {
        cv::Point3d w = origin - plane_center;
        double fac = -plane_normal.dot(w) / dot;
        cv::Point3d u = root_vector * fac;
        out_point = origin + u;
        return true;
    }
    return false;
}

cv::Point2d get_centroid(cv::InputArray points)
{
    cv::Moments mm = cv::moments(points, false);
    double moment10 = mm.m10;
    double moment01 = mm.m01;
    double moment00 = mm.m00;
    cv::Point2d coord;
    coord.x = moment10 / moment00;
    coord.y = moment01 / moment00;
    return coord;
}

cv::Point2d get_max_pt(std::vector<std::vector<cv::Point>> points)
{
    cv::Size max_size(0, 0);
    cv::Point max_pt(0, 0);
    for (size_t index = 0; index < points.size(); index++)
    {
        cv::Rect bbox = cv::boundingRect(points[index]);

        if (bbox.width * bbox.height > max_size.width * max_size.height)
        {
            max_pt.x = bbox.x;
            max_pt.y = bbox.y;
            max_size.width = bbox.width;
            max_size.height = bbox.height;
        }
    }

    return max_pt;
}

std::vector<std::vector<cv::Point>> get_cv_contours(std::vector<bw_interfaces::Contour> contours)
{
    std::vector<std::vector<cv::Point>> cv_contours = {std::vector<cv::Point>()};
    for (size_t contour_index = 0; contour_index < contours.size(); contour_index++)
    {
        bw_interfaces::Contour contour = contours[contour_index];
        for (size_t points_index = 0; points_index < contour.points.size(); points_index++)
        {
            int contour_x = (int)contour.points[points_index].x;
            int contour_y = (int)contour.points[points_index].y;
            cv::Point point(contour_x, contour_y);
            cv_contours[0].push_back(point);
        }
    }
    return cv_contours;
}
