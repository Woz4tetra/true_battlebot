#pragma once

#include <ros/ros.h>
#include <omp.h>

#include <opencv2/core.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/highgui.hpp>

#include <sensor_msgs/Image.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>

#include <image_geometry/pinhole_camera_model.h>

#include <cv_bridge/cv_bridge.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <bw_interfaces/SegmentationInstanceArray.h>
#include <bw_interfaces/SegmentationInstance.h>
#include <bw_interfaces/Contour.h>

#include "base_estimation.h"


class RecommendedPlanePoint : BaseEstimation
{
private:
    ros::Publisher _recommended_point_pub;
    ros::Publisher _recommended_marker_pub;

    bool find_recommended_point(geometry_msgs::PointStamped& point_msg, cv::Mat depth_image, cv::Mat mask);

    void synced_callback(
        const sensor_msgs::ImageConstPtr& depth_image,
        const bw_interfaces::SegmentationInstanceArrayConstPtr& segmentation);
    geometry_msgs::Point convertPointToMsg(const cv::Point2i point, float depth);
public:
    RecommendedPlanePoint(ros::NodeHandle* nodehandle);
    ~RecommendedPlanePoint();
    int run();
};


visualization_msgs::Marker point_to_marker(geometry_msgs::PointStamped point_msg)
{
    visualization_msgs::Marker marker;
    marker.header = point_msg.header;
    marker.ns = "recommended_point";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = point_msg.point;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    return marker;
}