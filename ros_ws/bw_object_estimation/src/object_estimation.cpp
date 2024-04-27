#include "object_estimation.h"

ObjectEstimation::ObjectEstimation(ros::NodeHandle *nodehandle) : BaseEstimation(nodehandle)
{
    _robot_pub = nh.advertise<bw_interfaces::EstimatedObjectArray>("estimation/robots", _queue_size);
    _robot_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("estimation/robot_markers", _queue_size);

    ROS_INFO("ObjectEstimation node initialized");
}

ObjectEstimation::~ObjectEstimation()
{
}

void ObjectEstimation::synced_callback(
    const sensor_msgs::ImageConstPtr &depth_image,
    const bw_interfaces::SegmentationInstanceArrayConstPtr &segmentation)
{
    cv::Mat depth_cv_image;
    if (!get_depth_image(_camera_model, depth_cv_image, depth_image))
    {
        return;
    }

    bw_interfaces::EstimatedObjectArray robot_array;
    visualization_msgs::MarkerArray robot_markers;

    for (size_t index = 0; index < segmentation->instances.size(); index++)
    {
        bw_interfaces::SegmentationInstance instance = segmentation->instances[index];
        if (!is_label_included(instance.label))
        {
            continue;
        }

        std::vector<std::vector<cv::Point>> cv_contours = get_cv_contours(instance.contours);

        if (cv_contours.size() == 0)
        {
            continue;
        }
        bw_interfaces::EstimatedObject robot_msg;
        if (!find_object(robot_msg, depth_cv_image, cv_contours))
        {
            continue;
        }
        robot_msg.label = instance.label;
        robot_msg.header = segmentation->header;
        robot_array.robots.push_back(robot_msg);

        fill_marker_array(instance.object_index, robot_msg, robot_markers);
    }
    _robot_pub.publish(robot_array);
    _robot_marker_pub.publish(robot_markers);
}

bool ObjectEstimation::find_object(bw_interfaces::EstimatedObject &robot_msg, cv::Mat depth_image, std::vector<std::vector<cv::Point>> cv_contours)
{
    cv::Point2d centroid(0, 0);
    size_t centroid_count = 0;
    for (size_t contour_index = 0; contour_index < cv_contours.size(); contour_index++)
    {
        if (cv_contours[contour_index].size() == 0)
        {
            continue;
        }
        cv::Point2d sub_centroid = get_centroid(cv_contours[contour_index]);
        if (std::isnan(sub_centroid.x) || std::isnan(sub_centroid.y))
        {
            continue;
        }
        centroid.x += sub_centroid.x;
        centroid.y += sub_centroid.y;
        centroid_count++;
    }
    if (centroid_count == 0)
    {
        return false;
    }
    centroid.x /= centroid_count;
    centroid.y /= centroid_count;

    cv::Point2d max_px(0, 0);
    for (size_t contour_index = 0; contour_index < cv_contours.size(); contour_index++)
    {
        for (size_t point_index = 0; point_index < cv_contours[contour_index].size(); point_index++)
        {
            cv::Point2d normalized_point(
                abs(cv_contours[contour_index][point_index].x - centroid.x),
                abs(cv_contours[contour_index][point_index].y - centroid.y));
            if (normalized_point.x > max_px.x)
            {
                max_px.x = normalized_point.x;
            }
            if (normalized_point.y > max_px.y)
            {
                max_px.y = normalized_point.y;
            }
        }
    }
    max_px.x += centroid.x;
    max_px.y += centroid.y;

    cv::Point3d center = get_ray(centroid, depth_image);

    robot_msg.pose.pose.position.x = center.x;
    robot_msg.pose.pose.position.y = center.y;
    robot_msg.pose.pose.position.z = center.z;
    robot_msg.pose.pose.orientation.w = 1.0; // orientation is not calculated

    cv::Point3d edge = get_ray(max_px, depth_image);

    robot_msg.size.x = abs(2 * (edge.x - center.x));
    robot_msg.size.y = abs(2 * (edge.y - center.y));
    robot_msg.size.z = abs(2 * (edge.z - center.z));

    return true;
}

void ObjectEstimation::fill_marker_array(int obj_index, bw_interfaces::EstimatedObject &robot_msg, visualization_msgs::MarkerArray &robot_markers)
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
    cube_marker.color.r = 1.0;
    cube_marker.color.g = 0.0;
    cube_marker.color.b = 0.0;
    robot_markers.markers.push_back(cube_marker);

    visualization_msgs::Marker arrow_marker;
    arrow_marker.header = robot_msg.header;
    arrow_marker.ns = robot_msg.label + "_arrow";
    arrow_marker.id = obj_index;
    arrow_marker.frame_locked = false;
    arrow_marker.lifetime = ros::Duration(1.0);
    arrow_marker.type = visualization_msgs::Marker::ARROW;
    arrow_marker.action = visualization_msgs::Marker::ADD;
    arrow_marker.pose = robot_msg.pose.pose;
    arrow_marker.scale.x = 0.5;
    arrow_marker.scale.y = 0.05;
    arrow_marker.scale.z = 0.05;
    arrow_marker.color.a = 1.0;
    arrow_marker.color.r = 1.0;
    arrow_marker.color.g = 0.1;
    arrow_marker.color.b = 0.1;
    robot_markers.markers.push_back(arrow_marker);

    visualization_msgs::Marker text_marker;
    text_marker.header = robot_msg.header;
    text_marker.ns = robot_msg.label + "_text";
    text_marker.id = obj_index;
    text_marker.frame_locked = false;
    text_marker.lifetime = ros::Duration(1.0);
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.pose = robot_msg.pose.pose;
    text_marker.pose.position.z -= 0.1;
    text_marker.scale.z = 0.1;
    text_marker.color.a = 1.0;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.text = robot_msg.label;
    robot_markers.markers.push_back(text_marker);
}

int ObjectEstimation::run()
{
    ros::spin();

    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_estimation");
    ros::NodeHandle nh;
    ObjectEstimation node(&nh);
    return node.run();
}
