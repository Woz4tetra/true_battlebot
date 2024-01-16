#pragma once

#include <ros/ros.h>
#include <omp.h>

#include <opencv2/core.hpp>

#include <std_msgs/Header.h>

#include <sensor_msgs/Image.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <image_geometry/pinhole_camera_model.h>

#include <cv_bridge/cv_bridge.h>

#include <bw_interfaces/SegmentationInstanceArray.h>
#include <bw_interfaces/SegmentationInstance.h>
#include <bw_interfaces/Contour.h>
#include <bw_interfaces/EstimatedObject.h>

#include "GRANSAC/PlaneModel.hpp"
#include "GRANSAC/GRANSAC.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <base_estimation.h>

class FieldEstimation
{
private:
    ros::NodeHandle nh; // ROS node handle

    std::vector<std::string> _include_labels;

    ros::Publisher _response_pub;
    ros::Subscriber _request_sub;
    ros::Subscriber _depth_info_sub;

    image_geometry::PinholeCameraModel _camera_model;
    sensor_msgs::CameraInfoPtr _info_msg;

    GRANSAC::RANSAC<PlaneModel, 3> *_estimator;

    bool find_plane(std_msgs::Header header, cv::Mat depth_image, cv::Mat mask, geometry_msgs::PoseStamped &plane_msg);

    void camera_info_callback(const sensor_msgs::CameraInfoConstPtr &camera_info);
    void plane_request_callback(const geometry_msgs::PointStampedConstPtr &request);

    bool is_label_included(std::string label);
    bool plane_fitting(const std::vector<Vector3VP> &points_input, double *center, double *normal);

public:
    FieldEstimation(ros::NodeHandle *nodehandle);
    ~FieldEstimation();
    int run();
};
