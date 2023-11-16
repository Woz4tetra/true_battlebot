#include <ros/ros.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <pluginlib/class_list_macros.h>

/**
 * @brief bw_recovery namespace contains the implementation of the FlailRecoveryBehavior class
 */
namespace bw_recovery
{
    /**
     * @class FlailRecoveryBehavior
     * @brief A custom RecoveryBehavior implementation that moves the robot away from the nearest obstacle
     * 
     * This class implements a recovery behavior that moves the robot away from the nearest obstacle detected in the global costmap.
     * The behavior calculates the safe distance based on the robot's footprint and publishes velocity commands until the robot reaches a safe distance.
     */
    class FlailRecoveryBehavior : public nav_core::RecoveryBehavior
    {
    public:
        /**
         * @brief Constructor
         */
        FlailRecoveryBehavior();

        /**
         * @brief Initialize the recovery behavior
         * 
         * @param name The name of the recovery behavior
         * @param tf A pointer to the tf2_ros::Buffer object
         * @param global_costmap A pointer to the global costmap
         * @param local_costmap A pointer to the local costmap
         */
        void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);

        /**
         * @brief Run the recovery behavior
         * 
         * This method contains the main control loop for the recovery behavior. It publishes velocity commands
         * to move the robot back and forth until it's away from obstacles or the timeout is reached.
         */
        void runBehavior();

    private:
        double frequency_;                  // Control loop frequency (Hz)
        double timeout_;                    // Maximum duration for the recovery behavior to run (seconds)
        double flail_speed_;                // Speed at which the robot moves back and forth (m/s)
        double flail_direction_interval_;   // Interval at which the robot changes direction (seconds)
        bool check_global_costmap_;         // Flag to check the global costmap for obstacles
        bool check_local_costmap_;          // Flag to check the local costmap for obstacles

        tf2_ros::Buffer* tf_;           //!< Pointer to the tf2_ros::Buffer object
        costmap_2d::Costmap2DROS* global_costmap_;  // Pointer to the global costmap
        costmap_2d::Costmap2DROS* local_costmap_;   // Pointer to the local costmap

        ros::Publisher cmd_vel_pub_;    // Publisher for the velocity commands

        /**
         * @brief Stop the robot by publishing a zero-velocity command
         * 
         * This method publishes a zero-velocity command to stop the robot's movement.
         */
        void stopRobot();

        /**
         * @brief Check if the robot is free to move
         * 
         * This method checks if the robot is free to move by checking the global and local costmap for obstacles.
        */
        bool isRobotFree();

        /**
         * @brief Check if the robot is in collision with a costmap
         * 
         * This method checks if the robot is in collision with a costmap by checking the supplied costmap for obstacles.
         * @param costmap Pointer to the costmap_2d::Costmap2D object representing a costmap
         * @return True if the robot is in collision with the costmap, false otherwise
        */
        bool isRobotFreeInCostmap(costmap_2d::Costmap2DROS *costmap);


        /**
         * @brief Transforms a given input polygon to the global frame
         *
         * @param input_polygon geometry_msgs::Polygon object representing the input polygon in the robot's frame
         * @param output_polygon Reference to a geometry_msgs::Polygon object where the transformed polygon will be stored
         * @param costmap Pointer to the costmap_2d::Costmap2DROS object representing a costmap
         * @return True if the transformation is successful, false otherwise
         */
        bool transformPolygonToGlobal(const geometry_msgs::Polygon *input_polygon, geometry_msgs::Polygon *output_polygon, costmap_2d::Costmap2DROS *costmap);

        /**
         * @brief Converts a polygon to a vector of costmap_2d::MapLocation objects
         *
         * @param polygon geometry_msgs::Polygon object representing the input polygon
         * @param costmap Pointer to the costmap_2d::Costmap2D object representing the global costmap
         * @return Vector of costmap_2d::MapLocation objects representing the cells within the polygon
         */
        std::vector<costmap_2d::MapLocation> convertPolygonToMapLocations(const geometry_msgs::Polygon *polygon, costmap_2d::Costmap2D *costmap);

    };
}
