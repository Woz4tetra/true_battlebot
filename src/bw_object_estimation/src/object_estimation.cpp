#include "object_estimation.h"

ObjectEstimation::ObjectEstimation(ros::NodeHandle *nodehandle) : BaseEstimation(nodehandle)
{
    _robot_pub = nh.advertise<bw_interfaces::EstimatedObjectArray>("estimation/robots", _queue_size);
    _robot_pose_pub = nh.advertise<geometry_msgs::PoseArray>("estimation/robot_poses", _queue_size);

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

    geometry_msgs::PoseArray robot_poses;
    robot_poses.header = segmentation->header;

    bw_interfaces::EstimatedObjectArray robot_array;

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
        robot_msg.state.header = segmentation->header;
        robot_array.robots.push_back(robot_msg);

        robot_poses.poses.push_back(robot_msg.state.pose.pose);
    }
    _robot_pub.publish(robot_array);
    _robot_pose_pub.publish(robot_poses);
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

    robot_msg.state.pose.pose.position.x = center.x;
    robot_msg.state.pose.pose.position.y = center.y;
    robot_msg.state.pose.pose.position.z = center.z;
    robot_msg.state.pose.pose.orientation.w = 1.0; // orientation is not calculated

    cv::Point3d edge = get_ray(max_px, depth_image);

    robot_msg.size.x = abs(2 * (edge.x - center.x));
    robot_msg.size.y = abs(2 * (edge.y - center.y));
    robot_msg.size.z = abs(2 * (edge.z - center.z));

    return true;
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
