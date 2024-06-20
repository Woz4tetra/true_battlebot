#include "depthai/depthai.hpp"
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/camera_publisher.h>
#include <image_transport/image_transport.h>

class DepthAiOak1W
{
private:
    ros::NodeHandle nh; // ROS node handle

    dai::ColorCameraProperties::SensorResolution _resolution;

    std::string _camera_name;
    cv_bridge::CvImage _img_bridge;
    image_transport::ImageTransport _image_transport;
    image_transport::CameraPublisher _camera_pub;

public:
    DepthAiOak1W(ros::NodeHandle *nodehandle);
    ~DepthAiOak1W();
    int run();
};
