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
        geometry_msgs::Pose pose = get_pose_from_points(front_point, back_point, get_plane_normal());

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

geometry_msgs::Pose KeypointToObject::get_pose_from_points(cv::Point3d front_point, cv::Point3d back_point, cv::Point3d plane_normal)
{
    Eigen::Vector3d eigen_front_point(front_point.x, front_point.y, front_point.z);
    Eigen::Vector3d eigen_back_point(back_point.x, back_point.y, back_point.z);
    Eigen::Vector3d eigen_plane_normal(plane_normal.x, plane_normal.y, plane_normal.z);

    // Calculate the direction vector from front_point to back_point
    Eigen::Vector3d direction = (eigen_front_point - eigen_back_point).normalized();

    // Get the plane normal

    // Calculate the rotation from the plane normal to the direction vector
    Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(eigen_plane_normal, direction);

    // Compute center point
    Eigen::Vector3d center = (eigen_front_point + eigen_back_point) / 2.0;

    // The pose is then defined by the front_point and the rotation
    Eigen::Affine3d pose = Eigen::Translation3d(center) * rotation;

    // Convert the pose to a geometry_msgs::Pose and return it
    geometry_msgs::Pose pose_msg;
    pose_msg.position.x = pose.translation()[0];
    pose_msg.position.y = pose.translation()[1];
    pose_msg.position.z = pose.translation()[2];
    pose_msg.orientation.w = rotation.w();
    pose_msg.orientation.x = rotation.x();
    pose_msg.orientation.y = rotation.y();
    pose_msg.orientation.z = rotation.z();

    return pose_msg;
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
