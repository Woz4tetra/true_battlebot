#include "bw_object_estimation.h"


ObjectEstimation::ObjectEstimation(ros::NodeHandle* nodehandle) :
    nh(*nodehandle)
{
    int queue_size;
    ros::param::param<int>("~queue_size", queue_size, 10);

    _depth_info_sub = nh.subscribe<sensor_msgs::CameraInfo>("depth/camera_info", 1, &ObjectEstimation::camera_info_callback, this);

    _depth_sub.subscribe(nh, "depth/image_raw", queue_size);
    _segmentation_sub.subscribe(nh, "segmentation", queue_size);

    _sync.reset(new Sync(ExactSyncPolicy(queue_size), _depth_sub, _segmentation_sub));
    _sync->registerCallback(boost::bind(&ObjectEstimation::synced_callback, this, _1, _2));

    ROS_INFO("ObjectEstimation node initialized");
}

ObjectEstimation::~ObjectEstimation()
{
}

void ObjectEstimation::camera_info_callback(const sensor_msgs::CameraInfoConstPtr& camera_info)
{
    _camera_model.fromCameraInfo(camera_info);
    _depth_info_sub.shutdown();
    ROS_INFO("Camera model loaded");
}

void ObjectEstimation::synced_callback(
    const sensor_msgs::ImageConstPtr& depth_image,
    const bw_interfaces::SegmentationInstanceArrayConstPtr& segmentation)
{
    if (!_camera_model.initialized())
    {
        ROS_WARN("Camera model not loaded yet");
        return;
    }
    
    cv_bridge::CvImagePtr depth_ptr;
    try {
        depth_ptr = cv_bridge::toCvCopy(depth_image);  // encoding: passthrough
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Failed to convert depth image: %s", e.what());
        return;
    }
    ROS_INFO("Received image and segmentation");
}


int ObjectEstimation::run()
{
    ros::spin();

    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_estimation");
    ros::NodeHandle nh;
    ObjectEstimation node(&nh);
    return node.run();
}
