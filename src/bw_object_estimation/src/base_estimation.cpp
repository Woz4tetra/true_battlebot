#include "base_estimation.h"

BaseEstimation::BaseEstimation(ros::NodeHandle *nodehandle) : nh(*nodehandle)
{
    ros::param::param<int>("~queue_size", _queue_size, 10);
    ros::param::param<std::vector<std::string>>("~include_labels", _include_labels, std::vector<std::string>());
    if (_include_labels.size() == 0)
    {
        ROS_INFO("No labels specified, including all labels");
    }
    _depth_msg = sensor_msgs::ImagePtr(new sensor_msgs::Image());

    _depth_info_sub = nh.subscribe<sensor_msgs::CameraInfo>("depth/camera_info", 1, &BaseEstimation::camera_info_callback, this);

    _depth_sub = nh.subscribe<sensor_msgs::Image>("depth/image_raw", 1, &BaseEstimation::depth_callback, this);
    _segmentation_sub = nh.subscribe<bw_interfaces::SegmentationInstanceArray>("segmentation", 1, &BaseEstimation::segmentation_callback, this);
}

BaseEstimation::~BaseEstimation()
{
}

void BaseEstimation::camera_info_callback(const sensor_msgs::CameraInfoConstPtr &camera_info)
{
    _camera_model.fromCameraInfo(camera_info);
    _depth_info_sub.shutdown();
    ROS_INFO("Camera model loaded");
}

void BaseEstimation::depth_callback(const sensor_msgs::ImageConstPtr &depth_image)
{
    *_depth_msg = *depth_image;
}

void BaseEstimation::segmentation_callback(const bw_interfaces::SegmentationInstanceArrayConstPtr &segmentation)
{
    if (_depth_msg != NULL && _depth_msg->header.frame_id.length() != 0)
    {
        synced_callback(_depth_msg, segmentation);
    }
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

cv::Point3d BaseEstimation::get_ray(cv::Point2d centroid_uv, cv::Mat depth_image)
{
    cv::Point3d ray = _camera_model.projectPixelTo3dRay(centroid_uv);
    int cx = std::min(depth_image.cols, std::max(0, (int)centroid_uv.x));
    int cy = std::min(depth_image.rows, std::max(0, (int)centroid_uv.y));
    float z_meters = depth_image.at<float>(cy, cx);
    float x_meters = ray.x * z_meters;
    float y_meters = ray.y * z_meters;
    return cv::Point3d(x_meters, y_meters, z_meters);
}

bool get_depth_image(image_geometry::PinholeCameraModel camera_model, cv::Mat &depth_image, const sensor_msgs::ImageConstPtr &depth_msg)
{
    if (!camera_model.initialized())
    {
        ROS_WARN("Camera model not loaded yet");
        return false;
    }

    cv_bridge::CvImagePtr depth_ptr;
    try
    {
        depth_ptr = cv_bridge::toCvCopy(depth_msg); // encoding: passthrough
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Failed to convert depth image: %s", e.what());
        return false;
    }
    depth_image = depth_ptr->image;

    double conversion = get_depth_conversion(depth_ptr->encoding);
    depth_image.convertTo(depth_image, CV_32F);
    depth_image *= conversion;
    cv::patchNaNs(depth_image, 0.0);
    if (depth_image.size() != camera_model.fullResolution())
    {
        ROS_WARN("Depth image size does not match camera model size");
        return false;
    }
    return true;
}

double get_depth_conversion(std::string encoding)
{
    switch (sensor_msgs::image_encodings::bitDepth(encoding))
    {
    case 8:
    case 16:
        return 0.001;
    case 32:
    case 64:
    default:
        return 1.0;
    };
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
