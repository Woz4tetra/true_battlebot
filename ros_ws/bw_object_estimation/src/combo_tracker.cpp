#include "combo_tracker.h"

ComboTracker::ComboTracker(ros::NodeHandle *nodehandle) : BaseEstimation(nodehandle)
{
    ros::param::param<double>("~z_limit", _z_limit, 0.2);
    ros::param::param<std::string>("~label", _label, "robot");
    XmlRpc::XmlRpcValue exclude_labels_param;
    std::string exclude_labels_key;
    if (ros::param::search("exclude_labels", exclude_labels_key))
    {
        nh.getParam(exclude_labels_key, exclude_labels_param);
        if (exclude_labels_param.getType() != XmlRpc::XmlRpcValue::Type::TypeArray ||
            exclude_labels_param.size() == 0)
        {
            throw std::runtime_error("exclude_labels is the wrong type or size");
        }
        for (int i = 0; i < exclude_labels_param.size(); i++)
        {
            if (exclude_labels_param[i].getType() != XmlRpc::XmlRpcValue::Type::TypeString)
            {
                throw std::runtime_error("exclude_labels contains non-string values");
            }
            _exclude_labels.push_back(static_cast<std::string>(exclude_labels_param[i]));
            ROS_INFO("Excluding label: %s", _exclude_labels.back().c_str());
        }
    }
    else
    {
        _exclude_labels = std::vector<std::string>();
    }

    int processing_width, processing_height;
    ros::param::param<bool>("~resize_image", _resize_image, true);
    ros::param::param<int>("~processing_width", processing_width, 960);
    ros::param::param<int>("~processing_height", processing_height, 540);
    _processing_size = cv::Size(processing_width, processing_height);

    double reset_cooldown;
    ros::param::param<double>("~reset_cooldown", reset_cooldown, 0.0);
    _reset_cooldown = ros::Duration(reset_cooldown);

    ros::param::param<double>("~min_track_size_px", _min_track_size_px, 5.0);
    _min_track_size_px /= 2;

    ros::param::param<int>("~morph_kernel_size", _morph_kernel_size, 3);
    ros::param::param<int>("~morph_iterations", _morph_iterations, 3);
    ros::param::param<int>("~min_area", _min_area, 25);
    ros::param::param<int>("~max_area", _max_area, 10000);
    ros::param::param<int>("~gaussian_kernel_size", _gaussian_kernel_size, 5);
    ros::param::param<double>("~learning_rate", _learning_rate, -1.0);
    ros::param::param<int>("~history_length", _history_length, 500);
    ros::param::param<int>("~var_threshold", _var_threshold, 16);

    ros::param::param<int>("~tracking_dilation_rate", _tracking_dilation_rate, 5);
    ros::param::param<int>("~post_contour_dilation", _post_contour_dilation, 5);

    _num_trackers = 0;
    _should_reset_tracker = false;
    _should_reset_background = true;

    _image_sub = nh.subscribe<sensor_msgs::Image>("image", 1, &ComboTracker::image_callback, this);
    _source_sub = nh.subscribe<bw_interfaces::EstimatedObjectArray>("source_robots", 1, &ComboTracker::source_callback, this);
    _debug_image_pub = nh.advertise<sensor_msgs::Image>("debug_image", 1);
    _robot_pub = nh.advertise<bw_interfaces::EstimatedObjectArray>("estimation/robots", 10);
    _robot_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("estimation/robot_markers", 10);

    _back_subtractor = cv::createBackgroundSubtractorMOG2(_history_length, _var_threshold, true);
    _morph_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(_morph_kernel_size, _morph_kernel_size));
    _processing_size = cv::Size(processing_width, processing_height);
}

ComboTracker::~ComboTracker()
{
}

void ComboTracker::source_callback(const bw_interfaces::EstimatedObjectArrayConstPtr &robots)
{
    ros::Time now = ros::Time::now();
    if (now - _last_reset_time < _reset_cooldown)
    {
        ROS_DEBUG("Ignoring source callback due to reset cooldown");
        return;
    }
    sensor_msgs::CameraInfo info = _camera_model.cameraInfo();

    if (info.header.frame_id.empty())
    {
        ROS_ERROR("Camera info frame_id is empty. Cannot project robots to camera.");
        return;
    }
    for (size_t index = _init_boxes.size(); index < robots->robots.size(); index++)
    {
        _init_boxes.push_back(cv::Rect2d());
    }
    _num_trackers = robots->robots.size();

    for (size_t index = 0; index < _num_trackers; index++)
    {
        if (std::find(_exclude_labels.begin(), _exclude_labels.end(), robots->robots[index].label) != _exclude_labels.end())
        {
            ROS_DEBUG("Excluding label %s", robots->robots[index].label.c_str());
            continue;
        }
        geometry_msgs::TransformStamped transform;
        try
        {
            transform = _tf_buffer.lookupTransform(info.header.frame_id, robots->robots[index].header.frame_id, ros::Time(0), ros::Duration(5.0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("Failed to look up transform from %s to %s. %s", info.header.frame_id.c_str(), robots->robots[index].header.frame_id.c_str(), ex.what());
            return;
        }

        // Transform the pose to the camera frame
        geometry_msgs::PoseStamped pose_in_camera;
        pose_in_camera.header = robots->robots[index].header;
        pose_in_camera.pose = robots->robots[index].pose.pose;
        tf2::doTransform(pose_in_camera, pose_in_camera, transform);

        double size = std::max(robots->robots[index].size.x, robots->robots[index].size.y);
        geometry_msgs::Point robot_pos = pose_in_camera.pose.position;
        cv::Point3d center_xyz = cv::Point3d(robot_pos.x, robot_pos.y, robot_pos.z);
        cv::Point3d edge_xyz = center_xyz + (size / 2) * cv::Point3d(1, 1, 0);
        cv::Point2d center_uv = _camera_model.project3dToPixel(center_xyz);
        cv::Point2d edge_uv = _camera_model.project3dToPixel(edge_xyz);
        double half_width = abs(edge_uv.x - center_uv.x);
        double half_height = abs(edge_uv.y - center_uv.y);

        cv::Rect2d bbox;
        bbox.x = center_uv.x - half_width;
        bbox.y = center_uv.y - half_height;
        bbox.width = half_width * 2;
        bbox.height = half_height * 2;

        _init_boxes[index] = bbox;
    }
    _should_reset_tracker = true;
    _last_reset_time = now;
    ROS_DEBUG("Initialized %d trackers", _num_trackers);
}

void ComboTracker::image_callback(const sensor_msgs::ImageConstPtr &image_msg)
{
    if (_num_trackers == 0)
    {
        return;
    }

    bool publish_debug_image = _debug_image_pub.getNumSubscribers() > 0;
    cv_bridge::CvImagePtr color_ptr;
    try
    {
        color_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Failed to convert color image: %s", e.what());
        return;
    }

    cv::Mat image = color_ptr->image;
    cv::Mat debug_image;
    cv::Mat processing_image;
    cv::resize(image, processing_image, _processing_size);

    double width_ratio = _resize_image ? (double)image.cols / _processing_size.width : 1.0;
    double height_ratio = _resize_image ? (double)image.rows / _processing_size.height : 1.0;

    if (_should_reset_tracker)
    {
        _tracking_mask = cv::Mat::zeros(processing_image.size(), CV_8UC1);
        for (size_t index = 0; index < _num_trackers; index++)
        {
            cv::Rect2d scaled_bbox;
            scaled_bbox.x = std::max(0.0, _init_boxes[index].x / width_ratio);
            scaled_bbox.y = std::max(0.0, _init_boxes[index].y / height_ratio);
            scaled_bbox.width = std::max(_init_boxes[index].width / width_ratio, _min_track_size_px);
            scaled_bbox.height = std::max(_init_boxes[index].height / height_ratio, _min_track_size_px);
            scaled_bbox.width = std::min(scaled_bbox.width, (double)processing_image.cols - scaled_bbox.x);
            scaled_bbox.height = std::min(scaled_bbox.height, (double)processing_image.rows - scaled_bbox.y);
            ROS_DEBUG("Initializing tracker %lu with bbox (%f, %f, %f, %f)", index, scaled_bbox.x, scaled_bbox.y, scaled_bbox.width, scaled_bbox.height);

            _tracking_mask(scaled_bbox) = 255;
        }
        _should_reset_tracker = false;
    }

    if (_resize_image)
    {
        cv::resize(image, processing_image, _processing_size);
    }
    else
    {
        processing_image = image;
    }

    if (publish_debug_image)
    {
        debug_image = processing_image.clone();
    }

    if (_tracking_mask.empty())
    {
        ROS_WARN("Tracking mask is empty, skipping frame");
        return;
    }

    cv::dilate(_tracking_mask, _tracking_mask, _morph_kernel, cv::Point(-1, -1), _tracking_dilation_rate);

    cv::GaussianBlur(processing_image, processing_image, cv::Size(_gaussian_kernel_size, _gaussian_kernel_size), 0);

    double learning_rate = _should_reset_background ? 1.0 : _learning_rate;
    if (_should_reset_background)
    {
        ROS_INFO("Resetting background model of motion tracker");
        _should_reset_background = false;
    }

    cv::Mat fg_mask;
    _back_subtractor->apply(processing_image, fg_mask, learning_rate);
    double shadow_threshold = _back_subtractor->getShadowThreshold();

    cv::threshold(fg_mask, fg_mask, (int)shadow_threshold + 1, 255, cv::THRESH_BINARY);
    cv::morphologyEx(fg_mask, fg_mask, cv::MORPH_OPEN, _morph_kernel, cv::Point(-1, -1), _morph_iterations);

    cv::bitwise_and(fg_mask, _tracking_mask, fg_mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(fg_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (publish_debug_image)
    {
        cv::drawContours(debug_image, contours, -1, cv::Scalar(0, 255, 0), 1);
        cv::Mat fg_mask_color;
        cv::cvtColor(fg_mask, fg_mask_color, cv::COLOR_GRAY2BGR);
        cv::addWeighted(debug_image, 0.7, fg_mask_color, 0.3, 0, debug_image);
    }

    cv::Point3d plane_center = get_plane_center();
    cv::Point3d plane_normal = get_plane_normal();

    bw_interfaces::EstimatedObjectArray robot_array;
    visualization_msgs::MarkerArray robot_markers;

    std::vector<cv::Rect> bounding_boxes;
    for (size_t index = 0; index < contours.size(); index++)
    {
        double area = cv::contourArea(contours[index]);
        if (!(_min_area < area && area < _max_area))
        {
            continue;
        }

        cv::Rect bbox = cv::boundingRect(contours[index]);
        bbox.x = std::max(0, std::min(processing_image.cols - 1, bbox.x));
        bbox.y = std::max(0, std::min(processing_image.rows - 1, bbox.y));
        bbox.width = std::min(std::max(1, processing_image.cols - bbox.x), bbox.width + 2 * _post_contour_dilation);
        bbox.height = std::min(std::max(1, processing_image.rows - bbox.y), bbox.height + 2 * _post_contour_dilation);
        bounding_boxes.push_back(bbox);
    }

    std::set<int> to_remove;
    for (size_t index = 0; index < bounding_boxes.size(); index++)
    {
        for (size_t other_index = 0; other_index < bounding_boxes.size(); other_index++)
        {
            if (index == other_index)
            {
                continue;
            }
            cv::Rect intersection = bounding_boxes[index] & bounding_boxes[other_index];
            if (intersection.area() > 0)
            {
                if (cv::contourArea(contours[index]) > cv::contourArea(contours[other_index]))
                {
                    to_remove.insert(other_index);
                }
                else
                {
                    to_remove.insert(index);
                }
            }
        }
    }
    for (std::set<int>::reverse_iterator it = to_remove.rbegin(); it != to_remove.rend(); it++)
    {
        bounding_boxes.erase(bounding_boxes.begin() + *it);
    }

    for (size_t index = 0; index < bounding_boxes.size(); index++)
    {
        cv::Rect bbox = bounding_boxes[index];
        cv::Point2d centroid_uv = cv::Point2d(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2);
        cv::Point2d max_pt = cv::Point2d(bbox.x, bbox.y);

        cv::Point2d centroid_scaled = cv::Point2d(centroid_uv.x * width_ratio, centroid_uv.y * height_ratio);
        cv::Point2d max_pt_scaled = cv::Point2d(max_pt.x * width_ratio, max_pt.y * height_ratio);

        if (publish_debug_image)
        {
            cv::circle(debug_image, centroid_uv, 3, cv::Scalar(0, 0, 255), -1);
            cv::circle(debug_image, max_pt, 3, cv::Scalar(255, 0, 0), -1);
        }

        cv::Point3d center;
        if (!project_to_field(centroid_scaled, plane_center, plane_normal, center))
        {
            ROS_WARN_THROTTLE(1.0, "Failed to project centroid to field");
            continue;
        }

        if (center.z < _z_limit)
        {
            ROS_DEBUG("Object too close, skipping");
            continue;
        }

        cv::Point3d edge;
        if (!project_to_field(max_pt_scaled, plane_center, plane_normal, edge))
        {
            ROS_WARN("Failed to project edge to field");
            continue;
        }

        bw_interfaces::EstimatedObject robot_msg;
        robot_msg.header = image_msg->header;
        robot_msg.label = _label;
        robot_msg.pose.pose.position.x = center.x;
        robot_msg.pose.pose.position.y = center.y;
        robot_msg.pose.pose.position.z = center.z;
        robot_msg.size.x = abs(2 * (edge.x - center.x));
        robot_msg.size.y = abs(2 * (edge.y - center.y));
        robot_msg.size.z = abs(2 * (edge.z - center.z));
        robot_msg.pose.pose.orientation.w = 1.0;
        robot_array.robots.push_back(robot_msg);

        fill_marker_array(index, robot_msg, robot_markers);
    }

    if (publish_debug_image)
    {
        cv_bridge::CvImage debug_image_msg;
        debug_image_msg.header = image_msg->header;
        debug_image_msg.encoding = sensor_msgs::image_encodings::BGR8;
        debug_image_msg.image = debug_image;
        _debug_image_pub.publish(debug_image_msg.toImageMsg());
    }

    _robot_pub.publish(robot_array);
    _robot_marker_pub.publish(robot_markers);
}

void ComboTracker::field_received_callback()
{
    _should_reset_background = true;
}

void ComboTracker::fill_marker_array(int obj_index, bw_interfaces::EstimatedObject &robot_msg, visualization_msgs::MarkerArray &robot_markers)
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
    cube_marker.color.r = 0.0;
    cube_marker.color.g = 0.0;
    cube_marker.color.b = 1.0;
    robot_markers.markers.push_back(cube_marker);

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
    robot_markers.markers.push_back(text_marker);
}

int ComboTracker::run()
{
    ros::spin();

    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "combo_tracker");
    ros::NodeHandle nh;
    ComboTracker node(&nh);
    return node.run();
}
