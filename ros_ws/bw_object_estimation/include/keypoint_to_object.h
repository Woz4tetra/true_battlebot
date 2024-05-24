#pragma once
#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

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
    std::string _front_keypoint_name;
    std::string _back_keypoint_name;

    ros::Subscriber _keypoint_sub;

    ros::Publisher _robot_pub;
    ros::Publisher _robot_marker_pub;

    bool is_label_included(std::string label);
    void fill_marker_array(int obj_index, bw_interfaces::EstimatedObject &robot_msg, visualization_msgs::MarkerArray &robot_markers);

    void keypoint_callback(const bw_interfaces::KeypointInstanceArrayConstPtr &keypoints);

    double get_label_height(std::string label);
    std::vector<cv::Point3d> project_keypoints_to_field(bw_interfaces::KeypointInstance instance);
    geometry_msgs::Pose get_pose_from_points(cv::Point3d front_point, cv::Point3d back_point, cv::Point3d plane_normal);

    void field_received_callback() {}

public:
    KeypointToObject(ros::NodeHandle *nodehandle);
    ~KeypointToObject();
    int run();
};

int get_index(const std::vector<std::string> vec, const std::string label)
{
    for (int i = 0; i < vec.size(); i++)
    {
        if (vec[i].compare(label) == 0)
        {
            return i;
        }
    }
    return -1;
}