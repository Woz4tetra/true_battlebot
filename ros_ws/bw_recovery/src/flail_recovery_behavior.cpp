#include "flail_recovery_behavior.h"

PLUGINLIB_EXPORT_CLASS(bw_recovery::FlailRecoveryBehavior, nav_core::RecoveryBehavior)


using namespace bw_recovery;


FlailRecoveryBehavior::FlailRecoveryBehavior() : local_costmap_(NULL)
{
}

void FlailRecoveryBehavior::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap)
{
    // Check if global and local costmaps are initialized
    if (!global_costmap || !local_costmap)
    {
        // If either costmap is not initialized, log an error and return
        ROS_ERROR("Global and local costmaps must be initialized.");
        return;
    }

    // Set the tf, global_costmap_, and local_costmap_ class members
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    // Create a private node handle with the given name
    ros::NodeHandle private_nh("~/" + name);

    // Load parameters from the parameter server
    private_nh.param("flail_speed", flail_speed_, 1.0);
    private_nh.param("flail_direction_interval", flail_direction_interval_, 1.0);
    private_nh.param("min_recovery_time", min_recovery_time_, 2.0);
    private_nh.param("timeout", timeout_, 10.0);
    private_nh.param("frequency", frequency_, 10.0);
    private_nh.param("check_global_costmap", check_global_costmap_, true);
    private_nh.param("check_local_costmap", check_local_costmap_, true);

    // Create a node handle for general use
    ros::NodeHandle nh;

    // Advertise the cmd_vel topic to publish velocity commands
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

void FlailRecoveryBehavior::runBehavior()
{
    ROS_WARN("Running flail recovery behavior");

    ros::Rate rate(frequency_);
    ros::Time now = ros::Time::now();
    ros::Time start_time = now;
    ros::Time direction_change_time = now;

    double x_velocity = flail_speed_;

    while (ros::ok() && now - start_time < ros::Duration(timeout_))
    {
        now = ros::Time::now();
        if (now - start_time > ros::Duration(min_recovery_time_) && isRobotFree()) {
            ROS_INFO("Robot is free, stopping flail recovery behavior");
            break;
        }

        if (now - direction_change_time > ros::Duration(flail_direction_interval_))
        {
            x_velocity = -x_velocity;
            direction_change_time = now;
        }

        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = x_velocity;
        cmd_vel.angular.z = 0;
        cmd_vel_pub_.publish(cmd_vel);

        rate.sleep();
    }

    stopRobot();
}

void FlailRecoveryBehavior::stopRobot()
{
    ROS_INFO("Setting robot velocity to zero.");
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    cmd_vel_pub_.publish(cmd_vel);
}

bool FlailRecoveryBehavior::isRobotFree()
{
    if (!check_global_costmap_ && !check_local_costmap_) {
        return false;
    }

    if (check_global_costmap_ && !isRobotFreeInCostmap(global_costmap_)) {
        return false;
    }

    if (check_local_costmap_ && !isRobotFreeInCostmap(local_costmap_)) {
        return false;
    }

    return true;
}

bool FlailRecoveryBehavior::isRobotFreeInCostmap(costmap_2d::Costmap2DROS *costmap_ros)
{
    geometry_msgs::Polygon robot_footprint = costmap_ros->getRobotFootprintPolygon();
    geometry_msgs::Polygon global_footprint;
    if (!transformPolygonToGlobal(&robot_footprint, &global_footprint, costmap_ros)) {
        return false;
    }

    costmap_2d::Costmap2D *costmap = costmap_ros->getCostmap();
    std::vector<costmap_2d::MapLocation> footprintMapLocations = convertPolygonToMapLocations(&global_footprint, costmap);
    std::vector<costmap_2d::MapLocation> cells;
    costmap->polygonOutlineCells(footprintMapLocations, cells);

    for (const auto& cell : cells)
    {
        if (costmap->getCost(cell.x, cell.y) == costmap_2d::LETHAL_OBSTACLE) {
            return false;
        }
    }
    return true;
}

bool FlailRecoveryBehavior::transformPolygonToGlobal(const geometry_msgs::Polygon *input_polygon, geometry_msgs::Polygon *output_polygon, costmap_2d::Costmap2DROS *costmap)
{
    // Get the global frame and robot base frame
    std::string global_frame = costmap->getGlobalFrameID();
    std::string robot_frame = costmap->getBaseFrameID();
    if (global_frame.compare(robot_frame) == 0) {
        output_polygon->points = input_polygon->points;
        return true;
    }

    // Look up the transform from robot_frame to global_frame
    geometry_msgs::TransformStamped transform;
    try
    {
        transform = tf_->lookupTransform(global_frame, robot_frame, ros::Time(0), ros::Duration(0.5));
    }
    catch (const tf2::TransformException& ex)
    {
        ROS_WARN_THROTTLE(1.0, "Failed to transform the polygon: %s", ex.what());
        return false;
    }

    // Iterate through the points in the input_polygon
    for (const geometry_msgs::Point32& input_point : input_polygon->points)
    {
        geometry_msgs::PointStamped input_point_stamped;
        geometry_msgs::PointStamped output_point_stamped;

        // Set the input point to a stamped version
        input_point_stamped.point.x = input_point.x;
        input_point_stamped.point.y = input_point.y;
        input_point_stamped.point.z = input_point.z;

        // Set the header frame_id and stamp
        input_point_stamped.header.frame_id = robot_frame;
        input_point_stamped.header.stamp = ros::Time::now();

        // Transform the point to the global frame using tf2::doTransform
        tf2::doTransform(input_point_stamped, output_point_stamped, transform);

        // Add the transformed point to the output polygon
        geometry_msgs::Point32 transformed_point;
        transformed_point.x = output_point_stamped.point.x;
        transformed_point.y = output_point_stamped.point.y;
        transformed_point.z = output_point_stamped.point.z;

        output_polygon->points.push_back(transformed_point);
    }

    return true;
}


std::vector<costmap_2d::MapLocation> FlailRecoveryBehavior::convertPolygonToMapLocations(const geometry_msgs::Polygon *polygon, costmap_2d::Costmap2D *costmap)
{
    std::vector<costmap_2d::MapLocation> mapLocations;

    // Iterate through the points in the polygon
    for (const auto& point : polygon->points)
    {
        // Convert the point's world coordinates to map coordinates in the costmap
        costmap_2d::MapLocation map_location;
        costmap->worldToMap(point.x, point.y, map_location.x, map_location.y);

        // Add the converted map location to the vector of map locations
        mapLocations.push_back(map_location);
    }

    // Return the vector of map locations corresponding to the polygon's points
    return mapLocations;
}
