#pragma once

#include <ros/ros.h>

#include <opencv2/core.hpp>

#include <sensor_msgs/Image.h>

#include <image_geometry/pinhole_camera_model.h>

#include <cv_bridge/cv_bridge.h>

#include <bw_interfaces/SegmentationInstanceArray.h>

class BaseEstimation
{
protected:
    ros::NodeHandle nh; // ROS node handle

    int _queue_size;
    std::vector<std::string> _include_labels;

    ros::Subscriber _depth_info_sub;
    ros::Subscriber _depth_sub;
    ros::Subscriber _segmentation_sub;

    image_geometry::PinholeCameraModel _camera_model;
    sensor_msgs::CameraInfoPtr _info_msg;
    sensor_msgs::ImagePtr _depth_msg;

    void camera_info_callback(const sensor_msgs::CameraInfoConstPtr &camera_info);
    void depth_callback(const sensor_msgs::ImageConstPtr &depth_image);
    void segmentation_callback(const bw_interfaces::SegmentationInstanceArrayConstPtr &segmentation);
    bool is_label_included(std::string label);
    cv::Point3d get_ray(cv::Point2d centroid, cv::Mat depth_image);

protected:
    virtual void synced_callback(
        const sensor_msgs::ImageConstPtr &depth_image,
        const bw_interfaces::SegmentationInstanceArrayConstPtr &segmentation) {}

public:
    BaseEstimation(ros::NodeHandle *nodehandle);
    ~BaseEstimation();
};

bool get_depth_image(image_geometry::PinholeCameraModel camera_model, cv::Mat &depth_image, const sensor_msgs::ImageConstPtr &depth_msg);
double get_depth_conversion(std::string encoding);
cv::Point2d get_centroid(cv::InputArray points);
std::vector<std::vector<cv::Point>> get_cv_contours(std::vector<bw_interfaces::Contour> contours);
