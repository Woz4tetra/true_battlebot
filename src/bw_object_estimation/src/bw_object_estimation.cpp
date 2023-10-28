#include "bw_object_estimation.h"


ObjectEstimation::ObjectEstimation(ros::NodeHandle* nodehandle) :
    nh(*nodehandle)
{
    int queue_size;
    ros::param::param<int>("~queue_size", queue_size, 10);

    _color_sub.subscribe(nh, "color/image_raw", queue_size);
    _depth_sub.subscribe(nh, "depth/image_raw", queue_size);
    _segmentation_sub.subscribe(nh, "segmentation", queue_size);

    _sync.reset(new Sync(ExactSyncPolicy(queue_size), _color_sub, _depth_sub, _segmentation_sub));
    _sync->registerCallback(boost::bind(&ObjectEstimation::synced_callback, this, _1, _2, _3));

    ROS_INFO("ObjectEstimation node initialized");
}

ObjectEstimation::~ObjectEstimation()
{
}

void ObjectEstimation::synced_callback(
    const sensor_msgs::ImageConstPtr& color_image,
    const sensor_msgs::ImageConstPtr& depth_image,
    const bw_interfaces::SegmentationInstanceArrayConstPtr& segmentation)
{
    ROS_INFO("Received RGBD image and segmentation");
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
