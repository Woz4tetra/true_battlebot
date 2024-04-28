#include "base_estimation.h"

BaseEstimation::BaseEstimation(ros::NodeHandle *nodehandle) : nh(*nodehandle), _tf_listener(_tf_buffer)
{
    ros::param::param<int>("~queue_size", _queue_size, 10);
    ros::param::param<std::vector<std::string>>("~include_labels", _include_labels, std::vector<std::string>());
    if (_include_labels.size() == 0)
    {
        ROS_INFO("No labels specified, including all labels");
    }

    _field_sub = nh.subscribe<bw_interfaces::EstimatedObject>("field", 1, &BaseEstimation::field_callback, this);
    _info_sub = nh.subscribe<sensor_msgs::CameraInfo>("camera_info", 1, &BaseEstimation::camera_info_callback, this);
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
    sensor_msgs::CameraInfo info = _camera_model.cameraInfo();
    geometry_msgs::TransformStamped transform;
    try
    {
        transform = _tf_buffer.lookupTransform(info.header.frame_id, field->header.frame_id, ros::Time(0), ros::Duration(15.0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("Failed to look up transform from %s to %s. %s", info.header.frame_id.c_str(), field->header.frame_id.c_str(), ex.what());
        return;
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
    _plane_normal = cv::Point3d(normal.x(), normal.y(), normal.z());

    ROS_INFO("Field received");
    _field_received = true;
}

bool BaseEstimation::is_label_included(std::string label)
{
    if (_include_labels.size() == 0)
    {
        return true;
    }
    else
    {
        for (size_t index = 0; index < _include_labels.size(); index++)
        {
            if (label.compare(_include_labels[index]) == 0)
            {
                return true;
            }
        }
        return false;
    }
}

bool BaseEstimation::project_to_field(cv::Point2d centroid_uv, cv::Point3d &out_point, double epsilon)
{
    cv::Point3d root_vector = _camera_model.projectPixelTo3dRay(centroid_uv);
    cv::Point3d origin = cv::Point3d(0, 0, 0);
    double dot = _plane_normal.dot(root_vector);
    if (std::abs(dot) > epsilon)
    {
        cv::Point3d w = origin - _plane_center;
        double fac = -_plane_normal.dot(w) / dot;
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

std::vector<std::vector<cv::Point>> get_cv_contours(std::vector<bw_interfaces::Contour> contours)
{
    std::vector<std::vector<cv::Point>> cv_contours = {std::vector<cv::Point>()};
    for (size_t contour_index = 0; contour_index < contours.size(); contour_index++)
    {
        bw_interfaces::Contour contour = contours[contour_index];
        for (size_t points_index = 0; points_index < contour.points.size(); points_index++)
        {
            int contour_x = contour.points[points_index].x;
            int contour_y = contour.points[points_index].y;
            cv::Point point(contour_x, contour_y);
            cv_contours[0].push_back(point);
        }
    }
    return cv_contours;
}
