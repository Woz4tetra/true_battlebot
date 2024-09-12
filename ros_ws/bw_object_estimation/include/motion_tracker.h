#pragma once
#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/video/background_segm.hpp>

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

    ros::Publisher _debug_image_pub;
    ros::Publisher _robot_pub;
    ros::Publisher _robot_marker_pub;

    bool _is_reset = false;

    double _z_limit;

    cv::Size _processing_size;

    int _morph_iterations;
    int _morph_kernel_size;

    int _min_area, _max_area;

    int _gaussian_kernel_size;

    double _learning_rate;
    int _history_length;
    int _var_threshold;

    std::string _label;

    cv::Ptr<cv::BackgroundSubtractorMOG2> _back_subtractor;
    cv::Mat _morph_kernel;

    void fill_marker_array(int obj_index, bw_interfaces::EstimatedObject &robot_msg, visualization_msgs::MarkerArray &robot_markers);

    void field_received_callback();
    void image_callback(const sensor_msgs::ImageConstPtr &image);

public:
    MotionTracker(ros::NodeHandle *nodehandle);
    ~MotionTracker();
    int run();
};
