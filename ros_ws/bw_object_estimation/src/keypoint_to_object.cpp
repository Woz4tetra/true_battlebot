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
        int front_index = get_index(instance.names, _front_keypoint_name);
        int back_index = get_index(instance.names, _back_keypoint_name);
        if (front_index == -1 || back_index == -1)
        {
            ROS_WARN("Front or back keypoint not found");
            continue;
        }
        cv::Point3d front_point = points[front_index];
        cv::Point3d back_point = points[back_index];
        geometry_msgs::Pose pose = get_pose_from_points(front_point, back_point);

        bw_interfaces::EstimatedObject robot_msg;
        robot_msg.label = instance.label;
        robot_msg.header = keypoints->header;
        robot_msg.pose.pose = pose;
        robot_array.robots.push_back(robot_msg);

        fill_marker_array(instance.object_index, robot_msg, robot_markers);
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

geometry_msgs::Pose KeypointToObject::get_pose_from_points(cv::Point3d front_point, cv::Point3d back_point)
{
    geometry_msgs::Pose pose;
    cv::Point3d direction = back_point - front_point;
    cv::Point3d center = (front_point + back_point) / 2;
    cv::Point3d up = cv::Point3d(0, 0, 1);
    cv::Point3d right = up.cross(direction);
    right = right / cv::norm(right);
    up = direction.cross(right);
    up = up / cv::norm(up);

    cv::Matx33d rotation(right.x, up.x, direction.x,
                         right.y, up.y, direction.y,
                         right.z, up.z, direction.z);
    cv::Vec3d rvec;
    cv::Rodrigues(rotation, rvec);
    pose.position.x = center.x;
    pose.position.y = center.y;
    pose.position.z = center.z;
    pose.orientation.x = rvec[0];
    pose.orientation.y = rvec[1];
    pose.orientation.z = rvec[2];
    pose.orientation.w = 1.0;
    return pose;
}

void KeypointToObject::fill_marker_array(int obj_index, bw_interfaces::EstimatedObject &robot_msg, visualization_msgs::MarkerArray &robot_markers)
{
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
