#pragma once
#include "ros/ros.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <image_geometry/pinhole_camera_model.h>

#include <cv_bridge/cv_bridge.h>

#include <bw_interfaces/SegmentationInstanceArray.h>


class ObjectEstimation
{
private:
    ros::NodeHandle nh;  // ROS node handle

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, bw_interfaces::SegmentationInstanceArray> ExactSyncPolicy;
    typedef message_filters::Synchronizer<ExactSyncPolicy> Sync;
    boost::shared_ptr<Sync> _sync;

    ros::Subscriber _depth_info_sub;
    message_filters::Subscriber<sensor_msgs::Image> _depth_sub;
    message_filters::Subscriber<bw_interfaces::SegmentationInstanceArray> _segmentation_sub;

    image_geometry::PinholeCameraModel _camera_model;

    void camera_info_callback(const sensor_msgs::CameraInfoConstPtr& camera_info);
    void synced_callback(
        const sensor_msgs::ImageConstPtr& depth_image,
        const bw_interfaces::SegmentationInstanceArrayConstPtr& segmentation);
public:
    ObjectEstimation(ros::NodeHandle* nodehandle);
    ~ObjectEstimation();
    int run();
};
