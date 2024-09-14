#pragma once
#include <ros/ros.h>

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/core.hpp>
#include <opencv2/tracking.hpp>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/Image.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <cv_bridge/cv_bridge.h>

#include <bw_interfaces/EstimatedObject.h>
#include <bw_interfaces/EstimatedObjectArray.h>

#include "base_estimation.h"

class ComboTracker : BaseEstimation
{
private:
    ros::NodeHandle nh; // ROS node handle

    ros::Subscriber _source_sub;
    ros::Subscriber _image_sub;

    ros::Publisher _debug_image_pub;
    ros::Publisher _robot_pub;
    ros::Publisher _robot_marker_pub;

    double _z_limit;
    std::string _label;
    bool _resize_image;
    cv::Size _processing_size;
    ros::Duration _reset_cooldown;
    ros::Time _last_reset_time;
    double _min_track_size_px;

    std::vector<cv::Rect2d> _init_boxes;
    std::vector<cv::Ptr<cv::Tracker>> _trackers;
    int _num_trackers;
    bool _should_reset;

    void fill_marker_array(int obj_index, bw_interfaces::EstimatedObject &robot_msg, visualization_msgs::MarkerArray &robot_markers);

    void field_received_callback();
    void image_callback(const sensor_msgs::ImageConstPtr &image);
    void source_callback(const bw_interfaces::EstimatedObjectArrayConstPtr &robots);

public:
    ComboTracker(ros::NodeHandle *nodehandle);
    ~ComboTracker();
    int run();
};
