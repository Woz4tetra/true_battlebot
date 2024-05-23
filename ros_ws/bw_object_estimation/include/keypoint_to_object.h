#pragma once
#include <ros/ros.h>

#include <opencv2/core.hpp>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <bw_interfaces/KeypointInstanceArray.h>
#include <bw_interfaces/KeypointInstance.h>
#include <bw_interfaces/EstimatedObject.h>
#include <bw_interfaces/EstimatedObjectArray.h>

#include "base_estimation.h"

class KeypointToObject : BaseEstimation
{
private:
    ros::NodeHandle nh; // ROS node handle

    std::vector<std::string> _include_labels;
    double _z_limit;

    ros::Subscriber _keypoint_sub;

    ros::Publisher _robot_pub;
    ros::Publisher _robot_marker_pub;

    bool is_label_included(std::string label);
    void fill_marker_array(int obj_index, bw_interfaces::EstimatedObject &robot_msg, visualization_msgs::MarkerArray &robot_markers);

    void keypoint_callback(const bw_interfaces::KeypointInstanceArrayConstPtr &keypoints);

    double get_label_height(std::string label);

    void field_received_callback() {}

public:
    KeypointToObject(ros::NodeHandle *nodehandle);
    ~KeypointToObject();
    int run();
};
