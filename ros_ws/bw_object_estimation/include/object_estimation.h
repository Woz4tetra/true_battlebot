#pragma once
#include <ros/ros.h>

#include <opencv2/core.hpp>

#include <sensor_msgs/Image.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <image_geometry/pinhole_camera_model.h>

#include <cv_bridge/cv_bridge.h>

#include <bw_interfaces/SegmentationInstanceArray.h>
#include <bw_interfaces/SegmentationInstance.h>
#include <bw_interfaces/Contour.h>
#include <bw_interfaces/EstimatedObject.h>
#include <bw_interfaces/EstimatedObjectArray.h>

#include "base_estimation.h"

class ObjectEstimation : BaseEstimation
{
private:
    ros::NodeHandle nh; // ROS node handle

    std::vector<std::string> _include_labels;

    ros::Subscriber _segmentation_sub;

    ros::Publisher _robot_pub;
    ros::Publisher _robot_marker_pub;

    bool is_label_included(std::string label);
    bool find_object(bw_interfaces::EstimatedObject &robot_msg, std::vector<std::vector<cv::Point>> cv_contours);
    void fill_marker_array(int obj_index, bw_interfaces::EstimatedObject &robot_msg, visualization_msgs::MarkerArray &robot_markers);

    void segmentation_callback(const bw_interfaces::SegmentationInstanceArrayConstPtr &segmentation);

public:
    ObjectEstimation(ros::NodeHandle *nodehandle);
    ~ObjectEstimation();
    int run();
};
