#include "segmentation_to_object.h"

SegmentationToObject::SegmentationToObject(ros::NodeHandle *nodehandle) : BaseEstimation(nodehandle)
{
    ros::param::param<double>("~z_limit", _z_limit, 0.2);
    ros::param::param<std::vector<std::string>>("~include_labels", _include_labels, std::vector<std::string>());
    if (_include_labels.size() == 0)
    {
        ROS_INFO("No labels specified, including all labels");
    }

    _segmentation_sub = nh.subscribe<bw_interfaces::SegmentationInstanceArray>("segmentation", 1, &SegmentationToObject::segmentation_callback, this);
    _robot_pub = nh.advertise<bw_interfaces::EstimatedObjectArray>("estimation/robots", 10);
    _robot_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("estimation/robot_markers", 10);

    ROS_INFO("SegmentationToObject node initialized");
}

SegmentationToObject::~SegmentationToObject()
{
}

bool SegmentationToObject::is_label_included(std::string label)
{
    if (_include_labels.size() == 0)
    {
        return true;
    }
    else
    {
        for (size_t index = 0; index < _include_labels.size(); index++)
        {
            if (label.compare(_include_labels[index]) == 0)
            {
                return true;
            }
        }
        return false;
    }
}

double SegmentationToObject::get_label_height(std::string label)
{
    if (_shared_config->labels.hasKey(label))
    {
        return _shared_config->labels.get(label).height;
    }
    return 0.0;
}

void SegmentationToObject::segmentation_callback(const bw_interfaces::SegmentationInstanceArrayConstPtr &segmentation)
{
    if (!is_field_received())
    {
        ROS_DEBUG_THROTTLE(1.0, "No field received, skipping estimation");
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
        robot_msg.label = instance.label;
        robot_msg.header = segmentation->header;
        if (!find_object(robot_msg, cv_contours))
        {
            continue;
        }
        robot_array.robots.push_back(robot_msg);

        fill_marker_array(instance.object_index, robot_msg, robot_markers);
    }
    _robot_pub.publish(robot_array);
    _robot_marker_pub.publish(robot_markers);
}

bool SegmentationToObject::find_object(bw_interfaces::EstimatedObject &robot_msg, std::vector<std::vector<cv::Point>> cv_contours)
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
        ROS_WARN("No valid centroid found");
        return false;
    }
    centroid.x /= centroid_count;
    centroid.y /= centroid_count;

    cv::Point2d max_pt = get_max_pt(cv_contours);

    cv::Point3d plane_center = get_plane_center();
    cv::Point3d plane_normal = get_plane_normal();

    cv::Point3d normal_offset = plane_normal * get_label_height(robot_msg.label);
    plane_center += normal_offset;

    cv::Point3d center;
    if (!project_to_field(centroid, plane_center, plane_normal, center))
    {
        ROS_WARN_THROTTLE(1.0, "Failed to project centroid to field");
        return false;
    }
    center -= normal_offset;

    if (center.z < _z_limit)
    {
        ROS_DEBUG("Object %s too close, skipping", robot_msg.label.c_str());
        return false;
    }

    robot_msg.pose.pose.position.x = center.x;
    robot_msg.pose.pose.position.y = center.y;
    robot_msg.pose.pose.position.z = center.z;
    robot_msg.pose.pose.orientation.w = 1.0; // orientation is not calculated

    cv::Point3d edge;
    if (!project_to_field(max_pt, plane_center, plane_normal, edge))
    {
        ROS_WARN("Failed to project edge to field");
        return false;
    }
    edge -= normal_offset;

    robot_msg.size.x = abs(2 * (edge.x - center.x));
    robot_msg.size.y = abs(2 * (edge.y - center.y));
    robot_msg.size.z = abs(2 * (edge.z - center.z));

    return true;
}

void SegmentationToObject::fill_marker_array(int obj_index, bw_interfaces::EstimatedObject &robot_msg, visualization_msgs::MarkerArray &robot_markers)
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
    arrow_marker.color.a = 0.75;
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
    text_marker.pose.position.y -= 0.1;
    text_marker.pose.position.z -= 0.1;
    text_marker.scale.z = 0.1;
    text_marker.color.a = 1.0;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.text = robot_msg.label;
    robot_markers.markers.push_back(text_marker);
}

int SegmentationToObject::run()
{
    ros::spin();

    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "segmentation_to_object");
    ros::NodeHandle nh;
    SegmentationToObject node(&nh);
    return node.run();
}
