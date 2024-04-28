#pragma once
#include <ros/ros.h>

#include <opencv2/core.hpp>

#include <sensor_msgs/Image.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <cv_bridge/cv_bridge.h>

#include <bw_interfaces/EstimatedObject.h>
#include <bw_interfaces/EstimatedObjectArray.h>

#include "base_estimation.h"

class MotionTracker : BaseEstimation
{
private:
    ros::NodeHandle nh; // ROS node handle

    ros::Subscriber _image_sub;

    void fill_marker_array(int obj_index, bw_interfaces::EstimatedObject &robot_msg, visualization_msgs::MarkerArray &robot_markers);

    void field_received_callback();
    void image_callback(const sensor_msgs::ImageConstPtr &image);

public:
    MotionTracker(ros::NodeHandle *nodehandle);
    ~MotionTracker();
    int run();
};
