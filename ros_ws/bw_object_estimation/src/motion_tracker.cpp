#include "motion_tracker.h"

MotionTracker::MotionTracker(ros::NodeHandle *nodehandle) : BaseEstimation(nodehandle)
{
    _image_sub = nh.subscribe<sensor_msgs::Image>("image_raw", 1, &MotionTracker::image_callback, this);
    _field_sub = nh.subscribe<bw_interfaces::EstimatedObject>("field", 1, &MotionTracker::field_callback, this);
}

void MotionTracker::image_callback(const sensor_msgs::ImageConstPtr &image)
{
    cv_bridge::CvImagePtr color_ptr;
    try
    {
        color_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Failed to convert color image: %s", e.what());
        return;
    }
}

void MotionTracker::field_received_callback()
{
}

void MotionTracker::fill_marker_array(int obj_index, bw_interfaces::EstimatedObject &robot_msg, visualization_msgs::MarkerArray &robot_markers)
{
    visualization_msgs::Marker sphere_marker;
    sphere_marker.header = robot_msg.header;
    sphere_marker.ns = robot_msg.label + "_sphere";
    sphere_marker.id = obj_index;
    sphere_marker.frame_locked = false;
    sphere_marker.lifetime = ros::Duration(1.0);
    sphere_marker.type = visualization_msgs::Marker::SPHERE;
    sphere_marker.action = visualization_msgs::Marker::ADD;
    sphere_marker.pose = robot_msg.pose.pose;
    sphere_marker.scale = robot_msg.size;
    sphere_marker.color.a = 0.5;
    sphere_marker.color.r = 1.0;
    sphere_marker.color.g = 0.0;
    sphere_marker.color.b = 0.0;
    robot_markers.markers.push_back(sphere_marker);

    visualization_msgs::Marker text_marker;
    text_marker.header = robot_msg.header;
    text_marker.ns = robot_msg.label + "_text";
    text_marker.id = obj_index;
    text_marker.frame_locked = false;
    text_marker.lifetime = ros::Duration(1.0);
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.pose = robot_msg.pose.pose;
    text_marker.pose.position.y -= 0.1;
    text_marker.pose.position.z -= 0.1;
    text_marker.scale.z = 0.1;
    text_marker.color.a = 1.0;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.text = robot_msg.label + "_" + std::to_string(obj_index);
    robot_markers.markers.push_back(text_marker);
}
