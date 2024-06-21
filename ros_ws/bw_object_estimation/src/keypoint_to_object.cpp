#include "keypoint_to_object.h"

KeypointToObject::KeypointToObject(ros::NodeHandle *nodehandle) : BaseEstimation(nodehandle)
{
    ros::param::param<double>("~z_limit", _z_limit, 0.2);
    ros::param::param<std::vector<std::string>>("~include_labels", _include_labels, std::vector<std::string>());
    ros::param::param<std::string>("~front_keypoint_name", _front_keypoint_name, "front");
    ros::param::param<std::string>("~back_keypoint_name", _back_keypoint_name, "back");
    if (_include_labels.size() == 0)
    {
        ROS_INFO("No labels specified, including all labels");
    }

    _keypoint_sub = nh.subscribe<bw_interfaces::KeypointInstanceArray>("keypoints", 1, &KeypointToObject::keypoint_callback, this);
    _robot_pub = nh.advertise<bw_interfaces::EstimatedObjectArray>("estimation/robots", 10);
    _robot_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("estimation/robot_markers", 10);

    ROS_INFO("KeypointToObject node initialized");
}

KeypointToObject::~KeypointToObject()
{
}

bool KeypointToObject::is_label_included(std::string label)
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

double KeypointToObject::get_label_height(std::string label)
{
    if (_shared_config->labels.hasKey(label))
    {
        return _shared_config->labels.get(label).height;
    }
    return 0.0;
}

void KeypointToObject::keypoint_callback(const bw_interfaces::KeypointInstanceArrayConstPtr &keypoints)
{
    if (!is_field_received())
    {
        ROS_WARN_THROTTLE(1.0, "No field received, skipping estimation");
        return;
    }

    bw_interfaces::EstimatedObjectArray robot_array;
    visualization_msgs::MarkerArray robot_markers;

    for (size_t index = 0; index < keypoints->instances.size(); index++)
    {
        bw_interfaces::KeypointInstance instance = keypoints->instances[index];
        if (!is_label_included(instance.label))
        {
            continue;
        }

        std::vector<cv::Point3d> points = project_keypoints_to_field(instance);
        int back_index = get_index(instance.names, _front_keypoint_name);
        int front_index = get_index(instance.names, _back_keypoint_name);
        if (front_index == -1 || back_index == -1)
        {
            ROS_WARN("Front or back keypoint not found");
            continue;
        }
        cv::Point3d front_point = points[front_index];
        cv::Point3d back_point = points[back_index];
        geometry_msgs::Pose pose = get_pose_from_points(front_point, back_point, get_plane_normal());

        bw_interfaces::EstimatedObject robot_msg;
        robot_msg.label = instance.label;
        robot_msg.header = keypoints->header;
        robot_msg.pose.pose = pose;
        robot_array.robots.push_back(robot_msg);

        fill_marker_array(instance.object_index, robot_msg, front_point, back_point, robot_markers);
    }
    _robot_pub.publish(robot_array);
    _robot_marker_pub.publish(robot_markers);
}

std::vector<cv::Point3d> KeypointToObject::project_keypoints_to_field(bw_interfaces::KeypointInstance instance)
{
    std::vector<cv::Point3d> points = std::vector<cv::Point3d>();
    cv::Point3d plane_center = get_plane_center();
    cv::Point3d plane_normal = get_plane_normal();

    for (size_t index = 0; index < instance.keypoints.size(); index++)
    {
        bw_interfaces::UVKeypoint keypoint = instance.keypoints[index];
        cv::Point2d pixel(keypoint.x, keypoint.y);
        cv::Point3d center;
        if (!project_to_field(pixel, plane_center, plane_normal, center))
        {
            ROS_WARN("Failed to project keypoint to field");
            continue;
        }
        points.push_back(center);
    }
    return points;
}

geometry_msgs::Pose KeypointToObject::get_pose_from_points(cv::Point3d front_point, cv::Point3d back_point, cv::Point3d plane_normal)
{
    Eigen::Vector3d eigen_front_point(front_point.x, front_point.y, front_point.z);
    Eigen::Vector3d eigen_back_point(back_point.x, back_point.y, back_point.z);
    Eigen::Vector3d origin_vec(1.0, 0.0, 0.0);

    // Calculate the direction vector from front_point to back_point
    Eigen::Vector3d direction = (eigen_front_point - eigen_back_point).normalized();

    // Calculate the rotation from the plane normal to the direction vector
    Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(origin_vec, direction);

    // Compute center point
    Eigen::Vector3d center = (eigen_front_point + eigen_back_point) / 2.0;

    // Convert the pose to a geometry_msgs::Pose and return it
    geometry_msgs::Pose pose_msg;
    pose_msg.position.x = center[0];
    pose_msg.position.y = center[1];
    pose_msg.position.z = center[2];
    pose_msg.orientation.w = rotation.w();
    pose_msg.orientation.x = rotation.x();
    pose_msg.orientation.y = rotation.y();
    pose_msg.orientation.z = rotation.z();

    return pose_msg;
}

void KeypointToObject::fill_marker_array(
    int obj_index,
    bw_interfaces::EstimatedObject &robot_msg,
    cv::Point3d front_point,
    cv::Point3d back_point,
    visualization_msgs::MarkerArray &markers)
{
    visualization_msgs::Marker front_marker;
    front_marker.header = robot_msg.header;
    front_marker.ns = robot_msg.label + "_front";
    front_marker.id = obj_index;
    front_marker.frame_locked = false;
    front_marker.lifetime = ros::Duration(1.0);
    front_marker.type = visualization_msgs::Marker::SPHERE;
    front_marker.action = visualization_msgs::Marker::ADD;
    front_marker.pose.position.x = front_point.x;
    front_marker.pose.position.y = front_point.y;
    front_marker.pose.position.z = front_point.z;
    front_marker.scale.x = 0.05;
    front_marker.scale.y = 0.05;
    front_marker.scale.z = 0.05;
    front_marker.color.a = 0.75;
    front_marker.color.r = 1.0;
    front_marker.color.g = 0.0;
    front_marker.color.b = 0.0;
    markers.markers.push_back(front_marker);

    visualization_msgs::Marker back_marker;
    back_marker.header = robot_msg.header;
    back_marker.ns = robot_msg.label + "_back";
    back_marker.id = obj_index;
    back_marker.frame_locked = false;
    back_marker.lifetime = ros::Duration(1.0);
    back_marker.type = visualization_msgs::Marker::SPHERE;
    back_marker.action = visualization_msgs::Marker::ADD;
    back_marker.pose.position.x = back_point.x;
    back_marker.pose.position.y = back_point.y;
    back_marker.pose.position.z = back_point.z;
    back_marker.scale.x = 0.05;
    back_marker.scale.y = 0.05;
    back_marker.scale.z = 0.05;
    back_marker.color.a = 0.75;
    back_marker.color.r = 0.0;
    back_marker.color.g = 0.0;
    back_marker.color.b = 1.0;
    markers.markers.push_back(back_marker);

    visualization_msgs::Marker line_marker;
    line_marker.header = robot_msg.header;
    line_marker.ns = robot_msg.label + "_line";
    line_marker.id = obj_index;
    line_marker.frame_locked = false;
    line_marker.lifetime = ros::Duration(1.0);
    line_marker.type = visualization_msgs::Marker::LINE_LIST;
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.points.push_back(front_marker.pose.position);
    line_marker.points.push_back(back_marker.pose.position);
    line_marker.scale.x = 0.01;
    line_marker.color.a = 0.75;
    line_marker.color.r = 0.5;
    line_marker.color.g = 0.5;
    line_marker.color.b = 0.5;
    markers.markers.push_back(line_marker);

    visualization_msgs::Marker pose_marker;
    pose_marker.header = robot_msg.header;
    pose_marker.ns = robot_msg.label + "_pose";
    pose_marker.id = obj_index;
    pose_marker.frame_locked = false;
    pose_marker.lifetime = ros::Duration(1.0);
    pose_marker.type = visualization_msgs::Marker::ARROW;
    pose_marker.action = visualization_msgs::Marker::ADD;
    pose_marker.pose = robot_msg.pose.pose;
    pose_marker.scale.x = 0.5;
    pose_marker.scale.y = 0.05;
    pose_marker.scale.z = 0.05;
    pose_marker.color.a = 0.75;
    pose_marker.color.r = 0.0;
    pose_marker.color.g = 1.0;
    pose_marker.color.b = 0.0;
    markers.markers.push_back(pose_marker);

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
    markers.markers.push_back(text_marker);
}

int KeypointToObject::run()
{
    ros::spin();

    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keypoint_to_object");
    ros::NodeHandle nh;
    KeypointToObject node(&nh);
    return node.run();
}
