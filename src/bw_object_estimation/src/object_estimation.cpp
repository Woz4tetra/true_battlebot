#include "object_estimation.h"


ObjectEstimation::ObjectEstimation(ros::NodeHandle* nodehandle) :
    BaseEstimation(nodehandle)
{
    _sync->registerCallback(boost::bind(&ObjectEstimation::synced_callback, this, _1, _2));

    _robot_pub = nh.advertise<bw_interfaces::EstimatedRobotArray>("estimation/robots", _queue_size);
    _robot_pose_pub = nh.advertise<geometry_msgs::PoseArray>("estimation/robot_poses", _queue_size);

    ROS_INFO("ObjectEstimation node initialized");
}

ObjectEstimation::~ObjectEstimation()
{
}

void ObjectEstimation::synced_callback(
    const sensor_msgs::ImageConstPtr& depth_image,
    const bw_interfaces::SegmentationInstanceArrayConstPtr& segmentation)
{
    cv::Mat depth_cv_image;
    if (!get_depth_image(depth_cv_image, depth_image)) {
        return;
    }

    geometry_msgs::PoseArray robot_poses;
    robot_poses.header = segmentation->header;

    bw_interfaces::EstimatedRobotArray robot_array;

    for (size_t index = 0; index < segmentation->instances.size(); index++)
    {
        bw_interfaces::SegmentationInstance instance = segmentation->instances[index];
        if (instance.label.compare(_robot_label) != 0) {
            continue;
        }

        std::vector<std::vector<cv::Point>> cv_contours = get_cv_contours(instance.contours);

        bw_interfaces::EstimatedRobot robot_msg = find_object(depth_cv_image, cv_contours);
        robot_msg.label = instance.label;
        robot_msg.header = segmentation->header;
        robot_array.robots.push_back(robot_msg);

        robot_poses.poses.push_back(robot_msg.pose);
    }
    _robot_pub.publish(robot_array);
    _robot_pose_pub.publish(robot_poses);
}

bw_interfaces::EstimatedRobot ObjectEstimation::find_object(cv::Mat depth_image, std::vector<std::vector<cv::Point>> cv_contours)
{
    cv::Point2d centroid(0.0, 0.0);
    for (size_t contour_index = 0; contour_index < cv_contours.size(); contour_index++)
    {
        cv::Point sub_centroid = get_centroid(cv_contours[contour_index]);
        centroid.x += sub_centroid.x;
        centroid.y += sub_centroid.y;
    }
    centroid.x /= cv_contours.size();
    centroid.y /= cv_contours.size();
    
    cv::Point3d ray = _camera_model.projectPixelTo3dRay(centroid);
    float z_meters = depth_image.at<float>((int)centroid.y, (int)centroid.x);
    float x_meters = ray.x * z_meters;
    float y_meters = ray.y * z_meters;

    bw_interfaces::EstimatedRobot robot_msg;

    robot_msg.pose.position.x = x_meters;
    robot_msg.pose.position.y = y_meters;
    robot_msg.pose.position.z = z_meters;
    robot_msg.pose.orientation.w = 1.0;  // orientation is not calculated

    return robot_msg;
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
