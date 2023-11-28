#pragma once

#include <ros/ros.h>
#include <omp.h>

#include <opencv2/core.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/highgui.hpp>

#include <std_msgs/Header.h>
#include <std_msgs/ColorRGBA.h>

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
    ros::Publisher _field_marker_pub;
    std_msgs::ColorRGBA _field_color;

    bool find_recommended_point(geometry_msgs::PointStamped& point_msg, cv::Mat depth_image, cv::Mat mask);
    geometry_msgs::Point convert_point_to_msg(const cv::Point2i point, float depth);
    std::vector<std::vector<geometry_msgs::Point>> project_contour_to_3d(cv::Mat depth_image, const bw_interfaces::SegmentationInstanceArrayConstPtr& segmentation);

protected:
    void synced_callback(
        const sensor_msgs::ImageConstPtr& depth_image,
        const bw_interfaces::SegmentationInstanceArrayConstPtr& segmentation);

public:
    RecommendedPlanePoint(ros::NodeHandle* nodehandle);
    ~RecommendedPlanePoint();
    int run();
};


visualization_msgs::Marker contour_to_marker(std_msgs::Header header, std::vector<geometry_msgs::Point> points_msg, std_msgs::ColorRGBA color, int marker_id = 0, double line_width = 0.01)
{
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns = "points";
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color = color;
    marker.points = points_msg;
    marker.frame_locked = false;
    marker.scale.x = 0.01;
    return marker;
}

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