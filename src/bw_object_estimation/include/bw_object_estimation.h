#pragma once
#include "ros/ros.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <bw_interfaces/SegmentationInstanceArray.h>


class ObjectEstimation
{
private:
    ros::NodeHandle nh;  // ROS node handle

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, bw_interfaces::SegmentationInstanceArray> ExactSyncPolicy;
    typedef message_filters::Synchronizer<ExactSyncPolicy> Sync;
    boost::shared_ptr<Sync> _sync;

    message_filters::Subscriber<sensor_msgs::Image> _color_sub;
    message_filters::Subscriber<sensor_msgs::Image> _depth_sub;
    message_filters::Subscriber<bw_interfaces::SegmentationInstanceArray> _segmentation_sub;

    void synced_callback(
        const sensor_msgs::ImageConstPtr& color_image,
        const sensor_msgs::ImageConstPtr& depth_image,
        const bw_interfaces::SegmentationInstanceArrayConstPtr& segmentation);
public:
    ObjectEstimation(ros::NodeHandle* nodehandle);
    ~ObjectEstimation();
    int run();
};
