#pragma once

#include <ros/ros.h>
#include <omp.h>

#include <opencv2/core.hpp>

#include <sensor_msgs/Image.h>

#include <geometry_msgs/PoseStamped.h>

#include <image_geometry/pinhole_camera_model.h>

#include <cv_bridge/cv_bridge.h>

#include <bw_interfaces/SegmentationInstanceArray.h>
#include <bw_interfaces/SegmentationInstance.h>
#include <bw_interfaces/Contour.h>
#include <bw_interfaces/EstimatedField.h>

#include "GRANSAC/PlaneModel.hpp"
#include "GRANSAC/GRANSAC.hpp"

#include "base_estimation.h"


class FieldEstimation : BaseEstimation
{
private:
    ros::Publisher _field_pub;
    ros::Publisher _field_pose_pub;

	GRANSAC::RANSAC<PlaneModel, 3>* _estimator;

    bw_interfaces::EstimatedField find_plane(cv::Mat depth_image, cv::Mat mask, std::vector<std::vector<cv::Point>> cv_contours);

    bool plane_fitting(const std::vector<Vector3VP> &points_input, double* center, double* normal);

    void synced_callback(
        const sensor_msgs::ImageConstPtr& depth_image,
        const bw_interfaces::SegmentationInstanceArrayConstPtr& segmentation);
public:
    FieldEstimation(ros::NodeHandle* nodehandle);
    ~FieldEstimation();
    int run();
};
