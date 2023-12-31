#pragma once
#include <ros/ros.h>

#include <opencv2/core.hpp>

#include <sensor_msgs/Image.h>

#include <geometry_msgs/PoseArray.h>

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
    ros::NodeHandle nh;  // ROS node handle

    ros::Publisher _robot_pub;
    ros::Publisher _robot_pose_pub;

    bool find_object(bw_interfaces::EstimatedObject& robot_msg, cv::Mat depth_image, std::vector<std::vector<cv::Point>> cv_contours);

protected:
    void synced_callback(
        const sensor_msgs::ImageConstPtr& depth_image,
        const bw_interfaces::SegmentationInstanceArrayConstPtr& segmentation);

public:
    ObjectEstimation(ros::NodeHandle* nodehandle);
    ~ObjectEstimation();
    int run();
};
