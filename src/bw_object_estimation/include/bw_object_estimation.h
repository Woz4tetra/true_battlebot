#pragma once
#include "ros/ros.h"

class ObjectEstimation
{
public:
    ObjectEstimation(ros::NodeHandle* nodehandle);
    ~ObjectEstimation();
    int run();
private:
    ros::NodeHandle nh;  // ROS node handle
    
};
