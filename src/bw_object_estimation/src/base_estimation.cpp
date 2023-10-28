#include "base_estimation.h"


BaseEstimation::BaseEstimation(ros::NodeHandle* nodehandle) :
    nh(*nodehandle)
{
    int queue_size;
    ros::param::param<int>("~queue_size", queue_size, 10);
    ros::param::param<std::string>("~field_label", _field_label, "field");

    _depth_info_sub = nh.subscribe<sensor_msgs::CameraInfo>("depth/camera_info", 1, &BaseEstimation::camera_info_callback, this);

    _depth_sub.subscribe(nh, "depth/image_raw", queue_size);
    _segmentation_sub.subscribe(nh, "segmentation", queue_size);

    _sync.reset(new Sync(ExactSyncPolicy(queue_size), _depth_sub, _segmentation_sub));
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
    return true;
}
