#include "recommended_plane_point.h"


RecommendedPlanePoint::RecommendedPlanePoint(ros::NodeHandle* nodehandle) :
    BaseEstimation(nodehandle)
{
    _sync->registerCallback(boost::bind(&RecommendedPlanePoint::synced_callback, this, _1, _2));

    _recommended_point_pub = nh.advertise<geometry_msgs::PointStamped>("estimation/recommended_field_point", _queue_size);
    _recommended_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("estimation/recommended_point_marker", _queue_size);

    cv::namedWindow("field_mask");
    ROS_INFO("RecommendedPlanePoint node initialized");
}

RecommendedPlanePoint::~RecommendedPlanePoint()
{
}

void RecommendedPlanePoint::synced_callback(
    const sensor_msgs::ImageConstPtr& depth_image,
    const bw_interfaces::SegmentationInstanceArrayConstPtr& segmentation)
{
    cv::Mat depth_cv_image;
    if (!get_depth_image(depth_cv_image, depth_image)) {
        return;
    }

    size_t num_contours = 0;
    cv::Mat field_mask = cv::Mat::zeros(depth_cv_image.rows, depth_cv_image.cols, CV_8UC1);
    for (size_t index = 0; index < segmentation->instances.size(); index++)
    {
        bw_interfaces::SegmentationInstance instance = segmentation->instances[index];
        if (instance.label.compare(_field_label) == 0) {
            std::vector<std::vector<cv::Point>> cv_contours = get_cv_contours(instance.contours);
            cv::drawContours(field_mask, cv_contours, -1, cv::Scalar(255), cv::FILLED);
            num_contours += cv_contours.size();
        }
    }

    for (size_t index = 0; index < segmentation->instances.size(); index++)
    {
        bw_interfaces::SegmentationInstance instance = segmentation->instances[index];
        if (instance.label.compare(_field_label) == 0) {
            continue;
        }

        std::vector<std::vector<cv::Point>> cv_contours = get_cv_contours(instance.contours);
        cv::drawContours(field_mask, cv_contours, -1, cv::Scalar(0), cv::FILLED);
    }

    if (num_contours > 0) {
        geometry_msgs::PointStamped point_msg;
        if (find_recommended_point(point_msg, depth_cv_image, field_mask)) {
            point_msg.header = segmentation->header;
            _recommended_point_pub.publish(point_msg);

            visualization_msgs::MarkerArray marker_array;
            marker_array.markers.push_back(point_to_marker(point_msg));
            _recommended_marker_pub.publish(marker_array);
        }
    }
}

bool RecommendedPlanePoint::find_recommended_point(geometry_msgs::PointStamped& point_msg, cv::Mat depth_image, cv::Mat mask) {
    if (cv::countNonZero(mask) == 0) {
        ROS_WARN("No white pixels in field mask");
        return false;
    }

    // If center is white, return the center point
    cv::Point2i center(depth_image.cols / 2, depth_image.rows / 2);
    if (mask.at<uchar>(center.y, center.x) != 0) {
        point_msg.point = convertPointToMsg(center, depth_image.at<float>(center.y, center.x));
        return true;
    }

    // Trace a black border around the mask
    cv::rectangle(mask, cv::Point(0, 0), cv::Point(mask.cols - 1, mask.rows - 1), cv::Scalar(0), 1);

    // Convert each pixel in mask to distance to nearest black pixel
    cv::Mat distance_mask;
    cv::distanceTransform(mask, distance_mask, cv::DIST_C, cv::DIST_MASK_PRECISE);

    // Find furthest point from black pixels
    cv::minMaxLoc(distance_mask, NULL, NULL, NULL, &center);

    // Convert the center point to a message and return it
    point_msg.point = convertPointToMsg(center, depth_image.at<float>((int)center.y, (int)center.x));
    return true;
}
geometry_msgs::Point RecommendedPlanePoint::convertPointToMsg(const cv::Point2i point, float depth)
{
    // Convert the pixel coordinates to meters
    cv::Point3d ray = _camera_model.projectPixelTo3dRay(cv::Point2d(point.x, point.y));
    float x_meters = ray.x * depth;
    float y_meters = ray.y * depth;

    // Create and return the point message
    geometry_msgs::Point point_msg;
    point_msg.x = x_meters;
    point_msg.y = y_meters;
    point_msg.z = depth;

    return point_msg;
}

int RecommendedPlanePoint::run()
{
    ros::spin();

    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "recommended_plane_point");
    ros::NodeHandle nh;
    RecommendedPlanePoint node(&nh);
    return node.run();
}
