#include <chrono>
#include <iostream>
#include <string>

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
    int _fps;
    double _alpha;

    std::string _camera_name;
    cv_bridge::CvImage _img_bridge;
    image_transport::ImageTransport _image_transport;
    image_transport::Publisher _rect_image_pub;
    image_transport::CameraPublisher _camera_pub;
    int _width, _height;

    ros::Time getFrameTime(ros::Time rosBaseTime,
                           std::chrono::time_point<std::chrono::steady_clock> steadyBaseTime,
                           std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> currTimePoint);
    void updateBaseTime(std::chrono::time_point<std::chrono::steady_clock> steadyBaseTime, ros::Time &rosBaseTime);
    sensor_msgs::CameraInfo createCameraInfo(int width, int height, std::vector<std::vector<float>> camIntrinsics, std::vector<float> distCoeffs);
    void resizeCameraInfo(sensor_msgs::CameraInfo &cameraInfoMsg, int destinationWidth, int destinationHeight);

public:
    DepthAiOak1W(ros::NodeHandle *nodehandle);
    ~DepthAiOak1W();
    int run();
};
