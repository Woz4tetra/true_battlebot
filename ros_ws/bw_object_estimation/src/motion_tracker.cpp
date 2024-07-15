#include "motion_tracker.h"

MotionTracker::MotionTracker(ros::NodeHandle *nodehandle) : BaseEstimation(nodehandle)
{
    ros::param::param<double>("~z_limit", _z_limit, 0.2);
    ros::param::param<double>("~learning_rate", _learning_rate, -1.0);
    int processing_width, processing_height;
    ros::param::param<int>("~processing_width", processing_width, 480);
    ros::param::param<int>("~processing_height", processing_height, 270);
    ros::param::param<int>("~blur_size", _blur_size, 3);
    ros::param::param<int>("~blur_iterations", _blur_iterations, 3);
    ros::param::param<int>("~min_area", _min_area, 25);
    ros::param::param<std::string>("~label", _label, "robot");

    _image_sub = nh.subscribe<sensor_msgs::Image>("image_raw", 1, &MotionTracker::image_callback, this);
    _debug_image_pub = nh.advertise<sensor_msgs::Image>("debug_image", 1);
    _robot_pub = nh.advertise<bw_interfaces::EstimatedObjectArray>("estimation/robots", 10);
    _robot_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("estimation/robot_markers", 10);

    _back_subtractor = cv::createBackgroundSubtractorMOG2();
    _blur_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(_blur_size, _blur_size));
    _processing_size = cv::Size(processing_width, processing_height);
}

MotionTracker::~MotionTracker()
{
}

void MotionTracker::image_callback(const sensor_msgs::ImageConstPtr &image_msg)
{
    bool publish_debug_image = _debug_image_pub.getNumSubscribers() > 0;
    cv_bridge::CvImagePtr color_ptr;
    try
    {
        color_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Failed to convert color image: %s", e.what());
        return;
    }

    cv::Mat image = color_ptr->image;
    cv::Mat processing_image;
    cv::resize(image, processing_image, _processing_size);
    double learning_rate = _is_reset ? _learning_rate : 1.0;
    if (!_is_reset)
    {
        ROS_INFO("Resetting background model of motion tracker");
    }
    _is_reset = true;

    cv::Mat fg_mask;
    _back_subtractor->apply(processing_image, fg_mask, learning_rate);
    cv::morphologyEx(fg_mask, fg_mask, cv::MORPH_OPEN, _blur_kernel, cv::Point(-1, -1), _blur_iterations);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(fg_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat debug_image;

    if (publish_debug_image)
    {
        cv::cvtColor(fg_mask, debug_image, cv::COLOR_GRAY2BGR);
    }

    cv::Point3d plane_center = get_plane_center();
    cv::Point3d plane_normal = get_plane_normal();

    bw_interfaces::EstimatedObjectArray robot_array;
    visualization_msgs::MarkerArray robot_markers;

    double width_ratio = (double)image.cols / _processing_size.width;
    double height_ratio = (double)image.rows / _processing_size.height;

    for (size_t index = 0; index < contours.size(); index++)
    {
        if (cv::contourArea(contours[index]) < _min_area)
        {
            continue;
        }

        cv::Point2d centroid_uv = get_centroid(contours[index]);
        cv::Rect bbox = cv::boundingRect(contours[index]);
        cv::Point2d max_pt = cv::Point2d(bbox.x, bbox.y);

        cv::Point2d centroid_scaled = cv::Point2d(centroid_uv.x * width_ratio, centroid_uv.y * height_ratio);
        cv::Point2d max_pt_scaled = cv::Point2d(max_pt.x * width_ratio, max_pt.y * height_ratio);

        if (publish_debug_image)
        {
            cv::drawContours(debug_image, contours, index, cv::Scalar(0, 255, 0), 1);
            cv::circle(debug_image, centroid_uv, 3, cv::Scalar(0, 0, 255), -1);
            cv::circle(debug_image, max_pt, 3, cv::Scalar(255, 0, 0), -1);
        }

        cv::Point3d center;
        if (!project_to_field(centroid_scaled, plane_center, plane_normal, center))
        {
            ROS_WARN_THROTTLE(1.0, "Failed to project centroid to field");
            continue;
        }

        if (center.z < _z_limit)
        {
            ROS_DEBUG("Object too close, skipping");
            continue;
        }

        cv::Point3d edge;
        if (!project_to_field(max_pt_scaled, plane_center, plane_normal, edge))
        {
            ROS_WARN("Failed to project edge to field");
            continue;
        }

        bw_interfaces::EstimatedObject robot_msg;
        robot_msg.header = image_msg->header;
        robot_msg.label = _label;
        robot_msg.pose.pose.position.x = center.x;
        robot_msg.pose.pose.position.y = center.y;
        robot_msg.pose.pose.position.z = center.z;
        robot_msg.size.x = abs(2 * (edge.x - center.x));
        robot_msg.size.y = abs(2 * (edge.y - center.y));
        robot_msg.size.z = abs(2 * (edge.z - center.z));
        robot_msg.pose.pose.orientation.w = 1.0;
        robot_array.robots.push_back(robot_msg);

        fill_marker_array(index, robot_msg, robot_markers);
    }

    if (publish_debug_image)
    {
        cv_bridge::CvImage debug_image_msg;
        debug_image_msg.header = image_msg->header;
        debug_image_msg.encoding = sensor_msgs::image_encodings::BGR8;
        debug_image_msg.image = debug_image;
        _debug_image_pub.publish(debug_image_msg.toImageMsg());
    }

    _robot_pub.publish(robot_array);
    _robot_marker_pub.publish(robot_markers);
}

void MotionTracker::field_received_callback()
{
    _is_reset = false;
}

void MotionTracker::fill_marker_array(int obj_index, bw_interfaces::EstimatedObject &robot_msg, visualization_msgs::MarkerArray &robot_markers)
{
    visualization_msgs::Marker cube_marker;
    cube_marker.header = robot_msg.header;
    cube_marker.ns = robot_msg.label + "_cube";
    cube_marker.id = obj_index;
    cube_marker.frame_locked = false;
    cube_marker.lifetime = ros::Duration(1.0);
    cube_marker.type = visualization_msgs::Marker::CUBE;
    cube_marker.action = visualization_msgs::Marker::ADD;
    cube_marker.pose = robot_msg.pose.pose;
    cube_marker.scale = robot_msg.size;
    cube_marker.color.a = 0.5;
    cube_marker.color.r = 0.0;
    cube_marker.color.g = 0.0;
    cube_marker.color.b = 1.0;
    robot_markers.markers.push_back(cube_marker);

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
    text_marker.color.a = 0.75;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.text = robot_msg.label + "_" + std::to_string(obj_index);
    robot_markers.markers.push_back(text_marker);
}

int MotionTracker::run()
{
    ros::spin();

    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_tracker");
    ros::NodeHandle nh;
    MotionTracker node(&nh);
    return node.run();
}
