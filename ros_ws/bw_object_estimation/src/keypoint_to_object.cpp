#include "keypoint_to_object.h"

KeypointToObject::KeypointToObject(ros::NodeHandle *nodehandle) : BaseEstimation(nodehandle)
{
    ros::param::param<double>("~z_limit", _z_limit, 0.2);
    ros::param::param<std::vector<std::string>>("~include_labels", _include_labels, std::vector<std::string>());
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

        // instance.keypoints

        bw_interfaces::EstimatedObject robot_msg;
        robot_msg.label = instance.label;
        robot_msg.header = keypoints->header;
        robot_array.robots.push_back(robot_msg);

        fill_marker_array(instance.object_index, robot_msg, robot_markers);
    }
    _robot_pub.publish(robot_array);
    _robot_marker_pub.publish(robot_markers);
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
