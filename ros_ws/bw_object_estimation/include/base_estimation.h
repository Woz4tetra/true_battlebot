#pragma once

#include <ros/ros.h>

#include <opencv2/core.hpp>

#include <Eigen/Geometry>

#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/Image.h>

#include <image_geometry/pinhole_camera_model.h>

#include <cv_bridge/cv_bridge.h>

#include <bw_interfaces/EstimatedObject.h>
#include <bw_interfaces/SegmentationInstanceArray.h>

class BaseEstimation
{
protected:
    ros::NodeHandle nh; // ROS node handle

    ros::Subscriber _info_sub;
    ros::Subscriber _field_sub;

    image_geometry::PinholeCameraModel _camera_model;
    sensor_msgs::CameraInfoPtr _info_msg;

    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;

    bool _field_received;
    cv::Point3d _plane_center, _plane_normal;

    void camera_info_callback(const sensor_msgs::CameraInfoConstPtr &camera_info);
    void field_callback(const bw_interfaces::EstimatedObjectConstPtr &field);
    bool project_to_field(cv::Point2d centroid_uv, cv::Point3d &out_point, double epsilon = 1e-6);
    bool is_field_received() { return _field_received; }

public:
    BaseEstimation(ros::NodeHandle *nodehandle);
    ~BaseEstimation();
};

cv::Point2d get_centroid(cv::InputArray points);
std::vector<std::vector<cv::Point>> get_cv_contours(std::vector<bw_interfaces::Contour> contours);
