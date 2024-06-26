#pragma once
#include <ros/ros.h>

#include <opencv2/core.hpp>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <bw_interfaces/SegmentationInstanceArray.h>
#include <bw_interfaces/SegmentationInstance.h>
#include <bw_interfaces/Contour.h>
#include <bw_interfaces/EstimatedObject.h>
#include <bw_interfaces/EstimatedObjectArray.h>

#include "base_estimation.h"

class SegmentationToObject : BaseEstimation
{
private:
    ros::NodeHandle nh; // ROS node handle

    std::vector<std::string> _include_labels;
    double _z_limit;

    ros::Subscriber _segmentation_sub;

    ros::Publisher _robot_pub;
    ros::Publisher _robot_marker_pub;

    bool is_label_included(std::string label);
    bool find_object(bw_interfaces::EstimatedObject &robot_msg, std::vector<std::vector<cv::Point>> cv_contours);
    void fill_marker_array(int obj_index, bw_interfaces::EstimatedObject &robot_msg, visualization_msgs::MarkerArray &robot_markers);

    void segmentation_callback(const bw_interfaces::SegmentationInstanceArrayConstPtr &segmentation);

    double get_label_height(std::string label);

    void field_received_callback() {}

public:
    SegmentationToObject(ros::NodeHandle *nodehandle);
    ~SegmentationToObject();
    int run();
};
