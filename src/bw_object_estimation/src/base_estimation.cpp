#include "base_estimation.h"


BaseEstimation::BaseEstimation(ros::NodeHandle* nodehandle) :
    nh(*nodehandle)
{
    ros::param::param<int>("~queue_size", _queue_size, 10);
    ros::param::param<std::vector<std::string>>("~include_labels", _include_labels, std::vector<std::string>());
    if (_include_labels.size() == 0) {
        ROS_INFO("No labels specified, including all labels");
    }

    _depth_info_sub = nh.subscribe<sensor_msgs::CameraInfo>("depth/camera_info", 1, &BaseEstimation::camera_info_callback, this);

    _depth_sub.subscribe(nh, "depth/image_raw", _queue_size);
    _segmentation_sub.subscribe(nh, "segmentation", _queue_size);

    _sync.reset(new Sync(ExactSyncPolicy(_queue_size), _depth_sub, _segmentation_sub));
}

BaseEstimation::~BaseEstimation()
{
}

void BaseEstimation::camera_info_callback(const sensor_msgs::CameraInfoConstPtr& camera_info)
{
    _camera_model.fromCameraInfo(camera_info);
    _depth_info_sub.shutdown();
    ROS_INFO("Camera model loaded");
}

bool BaseEstimation::get_depth_image(cv::Mat& depth_image, const sensor_msgs::ImageConstPtr& depth_msg)
{
    if (!_camera_model.initialized())
    {
        ROS_WARN("Camera model not loaded yet");
        return false;
    }
    
    cv_bridge::CvImagePtr depth_ptr;
    try {
        depth_ptr = cv_bridge::toCvCopy(depth_msg);  // encoding: passthrough
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Failed to convert depth image: %s", e.what());
        return false;
    }
    depth_image = depth_ptr->image;

    double conversion = get_depth_conversion(depth_ptr->encoding);
    depth_image.convertTo(depth_image, CV_32F);
    depth_image *= conversion;    
    cv::patchNaNs(depth_image, 0.0);
    if (depth_image.size() != _camera_model.fullResolution()) {
        ROS_WARN("Depth image size does not match camera model size");
        return false;
    }
    return true;
}


bool BaseEstimation::is_label_included(std::string label) {
    if (_include_labels.size() == 0) {
        return true;
    }
    else {
        for (size_t index = 0; index < _include_labels.size(); index++)
        {
            if (label.compare(_include_labels[index]) == 0) {
                return true;
            }
        }
        return false;
    }
}

cv::Point3d BaseEstimation::get_ray(cv::Point2d centroid_uv, cv::Mat depth_image) {
    cv::Point3d ray = _camera_model.projectPixelTo3dRay(centroid_uv);
    float z_meters = depth_image.at<float>((int)centroid_uv.y, (int)centroid_uv.x);
    float x_meters = ray.x * z_meters;
    float y_meters = ray.y * z_meters;
    return cv::Point3d(x_meters, y_meters, z_meters);
}
