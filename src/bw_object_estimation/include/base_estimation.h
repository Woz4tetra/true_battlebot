#pragma once

#include <ros/ros.h>

#include <opencv2/core.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <image_geometry/pinhole_camera_model.h>

#include <cv_bridge/cv_bridge.h>

#include <bw_interfaces/SegmentationInstanceArray.h>

class BaseEstimation
{
protected:
    ros::NodeHandle nh;  // ROS node handle

    int _queue_size;
    std::string _field_label;
    std::string _robot_label;

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, bw_interfaces::SegmentationInstanceArray> ExactSyncPolicy;
    typedef message_filters::Synchronizer<ExactSyncPolicy> Sync;
    boost::shared_ptr<Sync> _sync;

    ros::Subscriber _depth_info_sub;
    message_filters::Subscriber<sensor_msgs::Image> _depth_sub;
    message_filters::Subscriber<bw_interfaces::SegmentationInstanceArray> _segmentation_sub;

    image_geometry::PinholeCameraModel _camera_model;

    void camera_info_callback(const sensor_msgs::CameraInfoConstPtr& camera_info);
    bool get_depth_image(cv::Mat& depth_image, const sensor_msgs::ImageConstPtr& depth_msg);
public:
    BaseEstimation(ros::NodeHandle* nodehandle);
    ~BaseEstimation();
};

double get_depth_conversion(std::string encoding)
{
  switch (sensor_msgs::image_encodings::bitDepth(encoding)) {
    case 8:
    case 16:
      return 0.001;
    case 32:
    case 64:
    default:
      return 1.0;
  };
}

cv::Point get_centroid(cv::InputArray points)
{
    cv::Moments mm = cv::moments(points, false);
    double moment10 = mm.m10;
    double moment01 = mm.m01;
    double moment00 = mm.m00;
    cv::Point coord;
    coord.x = (int)(moment10 / moment00);
    coord.y = (int)(moment01 / moment00);
    return coord;
}


std::vector<std::vector<cv::Point>> get_cv_contours(std::vector<bw_interfaces::Contour> contours)
{
    std::vector<std::vector<cv::Point>> cv_contours = {std::vector<cv::Point>()};
    for (size_t contour_index = 0; contour_index < contours.size(); contour_index++)
    {
        bw_interfaces::Contour contour = contours[contour_index];
        for (size_t points_index = 0; points_index < contour.points.size(); points_index++)
        {
            int contour_x = contour.points[points_index].x;
            int contour_y = contour.points[points_index].y;
            cv::Point point(contour_x, contour_y);
            cv_contours[0].push_back(point);
        }
    }
    return cv_contours;
}

