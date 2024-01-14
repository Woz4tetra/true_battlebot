#include "recommended_plane_point.h"

RecommendedPlanePoint::RecommendedPlanePoint(ros::NodeHandle *nodehandle) : BaseEstimation(nodehandle)
{
    _recommended_point_pub = nh.advertise<geometry_msgs::PointStamped>("estimation/recommended_field_point", 1);
    _recommended_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("estimation/recommended_point_marker", 1);
    _field_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("estimation/field_marker", 1);

    _field_color.g = 1.0;
    _field_color.a = 1.0;

    _recommended_point.point.z = 1.0;

    ROS_INFO("RecommendedPlanePoint node initialized");
}

RecommendedPlanePoint::~RecommendedPlanePoint()
{
}

void RecommendedPlanePoint::synced_callback(
    const sensor_msgs::ImageConstPtr &depth_image,
    const bw_interfaces::SegmentationInstanceArrayConstPtr &segmentation)
{
    cv::Mat depth_cv_image;
    if (!get_depth_image(depth_cv_image, depth_image))
    {
        return;
    }

    size_t num_contours = 0;
    cv::Mat field_mask = cv::Mat::zeros(depth_cv_image.rows, depth_cv_image.cols, CV_8UC1);
    for (size_t index = 0; index < segmentation->instances.size(); index++)
    {
        bw_interfaces::SegmentationInstance instance = segmentation->instances[index];
        if (is_label_included(instance.label))
        {
            std::vector<std::vector<cv::Point>> cv_contours = get_cv_contours(instance.contours);
            cv::drawContours(field_mask, cv_contours, -1, cv::Scalar(255), cv::FILLED);
            num_contours += cv_contours.size();
        }
    }

    for (size_t index = 0; index < segmentation->instances.size(); index++)
    {
        bw_interfaces::SegmentationInstance instance = segmentation->instances[index];
        if (is_label_included(instance.label))
        {
            continue;
        }

        std::vector<std::vector<cv::Point>> cv_contours = get_cv_contours(instance.contours);
        cv::drawContours(field_mask, cv_contours, -1, cv::Scalar(0), cv::FILLED);
    }

    if (num_contours > 0)
    {
        find_recommended_point(_recommended_point, depth_cv_image, field_mask);

        if (_field_marker_pub.getNumSubscribers() > 0)
        {
            std::vector<std::vector<geometry_msgs::Point>> points = project_contour_to_3d(depth_cv_image, segmentation);
            visualization_msgs::MarkerArray marker_array;
            for (size_t index = 0; index < points.size(); index++)
            {
                marker_array.markers.push_back(contour_to_marker(segmentation->header, points[index], _field_color, index));
            }
            _field_marker_pub.publish(marker_array);
        }
    }
    _recommended_point.header = segmentation->header;
    _recommended_point_pub.publish(_recommended_point);

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.push_back(point_to_marker(_recommended_point));
    _recommended_marker_pub.publish(marker_array);
}

std::vector<std::vector<geometry_msgs::Point>> RecommendedPlanePoint::project_contour_to_3d(cv::Mat depth_image, const bw_interfaces::SegmentationInstanceArrayConstPtr &segmentation)
{
    std::vector<std::vector<geometry_msgs::Point>> all_points;
    for (size_t index = 0; index < segmentation->instances.size(); index++)
    {
        std::vector<geometry_msgs::Point> points;
        bw_interfaces::SegmentationInstance instance = segmentation->instances[index];
        if (!is_label_included(instance.label))
        {
            continue;
        }

        std::vector<std::vector<cv::Point>> cv_contours = get_cv_contours(instance.contours);
        for (size_t contour_index = 0; contour_index < cv_contours.size(); contour_index++)
        {
            std::vector<cv::Point> cv_contour = cv_contours[contour_index];
            for (size_t point_index = 0; point_index < cv_contour.size(); point_index++)
            {
                cv::Point2i point = cv_contour[point_index];
                geometry_msgs::Point point_msg = convert_point_to_msg(point, depth_image.at<float>(point.y, point.x));
                points.push_back(point_msg);
            }
        }
        all_points.push_back(points);
    }

    return all_points;
}

bool RecommendedPlanePoint::find_recommended_point(geometry_msgs::PointStamped &point_msg, cv::Mat depth_image, cv::Mat mask)
{
    if (cv::countNonZero(mask) == 0)
    {
        ROS_WARN("No white pixels in field mask");
        return false;
    }

    // If center is white, return the center point
    cv::Point2i center(depth_image.cols / 2, depth_image.rows / 2);
    if (mask.at<uchar>(center.y, center.x) != 0)
    {
        point_msg.point = convert_point_to_msg(center, depth_image.at<float>(center.y, center.x));
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
    point_msg.point = convert_point_to_msg(center, depth_image.at<float>((int)center.y, (int)center.x));
    return true;
}
geometry_msgs::Point RecommendedPlanePoint::convert_point_to_msg(const cv::Point2i point, float depth)
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
