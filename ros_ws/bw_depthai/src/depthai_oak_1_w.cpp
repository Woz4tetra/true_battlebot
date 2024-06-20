#include "depthai_oak_1_w.h"

DepthAiOak1W::DepthAiOak1W(ros::NodeHandle *nodehandle) : nh(*nodehandle),
                                                          _image_transport(nh)
{
    std::string resolution_name;
    ros::param::param<std::string>("~resolution", resolution_name, "1080_P");
    ros::param::param<std::string>("~camera_name", _camera_name, "oak");
    int queue_size;
    ros::param::param<int>("~queue_size", queue_size, 10);
    ros::param::param<int>("~fps", _fps, 60);
    ros::param::param<double>("~camera_matrix_alpha", _alpha, 0.0);

    if (resolution_name.compare("1080_P") == 0)
    {
        _resolution = dai::ColorCameraProperties::SensorResolution::THE_1080_P;
        _width = 1920;
        _height = 1080;
    }
    else if (resolution_name.compare("4_K") == 0)
    {
        _resolution = dai::ColorCameraProperties::SensorResolution::THE_4_K;
        _width = 3840;
        _height = 2160;
    }
    else if (resolution_name.compare("12_MP") == 0)
    {
        _resolution = dai::ColorCameraProperties::SensorResolution::THE_12_MP;
        _width = 4056;
        _height = 3040;
    }
    else if (resolution_name.compare("1352X1012") == 0)
    {
        _resolution = dai::ColorCameraProperties::SensorResolution::THE_1352X1012;
        _width = 1352;
        _height = 1012;
    }
    else if (resolution_name.compare("2024X1520") == 0)
    {
        _resolution = dai::ColorCameraProperties::SensorResolution::THE_2024X1520;
        _width = 2024;
        _height = 1520;
    }
    else
    {
        ROS_ERROR("Invalid resolution name: %s", resolution_name.c_str());
        ros::shutdown();
    }

    _camera_pub = _image_transport.advertiseCamera(_camera_name + "/image_raw", queue_size);
    _rect_image_pub = _image_transport.advertise(_camera_name + "/image_rect", queue_size);
}

DepthAiOak1W::~DepthAiOak1W()
{
}

ros::Time DepthAiOak1W::getFrameTime(ros::Time rosBaseTime,
                                     std::chrono::time_point<std::chrono::steady_clock> steadyBaseTime,
                                     std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> currTimePoint)
{
    auto elapsedTime = currTimePoint - steadyBaseTime;
    uint64_t nSec = rosBaseTime.toNSec() + std::chrono::duration_cast<std::chrono::nanoseconds>(elapsedTime).count();
    auto currTime = rosBaseTime;
    auto rosStamp = currTime.fromNSec(nSec);
    return rosStamp;
}

void DepthAiOak1W::updateBaseTime(std::chrono::time_point<std::chrono::steady_clock> steadyBaseTime, ros::Time &rosBaseTime)
{
    ros::Time currentRosTime = ros::Time::now();
    std::chrono::time_point<std::chrono::steady_clock> currentSteadyTime = std::chrono::steady_clock::now();
    // In nanoseconds
    auto expectedOffset = std::chrono::duration_cast<std::chrono::nanoseconds>(currentSteadyTime - steadyBaseTime).count();
    uint64_t previousBaseTimeNs = rosBaseTime.toNSec();
    rosBaseTime = rosBaseTime.fromNSec(currentRosTime.toNSec() - expectedOffset);
}

sensor_msgs::CameraInfo DepthAiOak1W::createCameraInfo(int width, int height, std::vector<std::vector<float>> camIntrinsics, std::vector<float> distCoeffs)
{
    sensor_msgs::CameraInfo cameraData;
    cameraData.width = width;
    cameraData.height = height;
    cameraData.header.frame_id = _camera_name;

    std::vector<double> flatIntrinsics, distCoeffsDouble;

    flatIntrinsics.resize(9);
    for (int i = 0; i < 3; i++)
    {
        std::copy(camIntrinsics[i].begin(), camIntrinsics[i].end(), flatIntrinsics.begin() + 3 * i);
    }

    auto &intrinsics = cameraData.K;
    auto &distortions = cameraData.D;
    auto &projection = cameraData.P;
    auto &rotation = cameraData.R;
    // Set rotation to reasonable default even for non-stereo pairs
    rotation[0] = rotation[4] = rotation[8] = 1;
    for (size_t i = 0; i < 3; i++)
    {
        std::copy(flatIntrinsics.begin() + i * 3, flatIntrinsics.begin() + (i + 1) * 3, projection.begin() + i * 4);
    }
    std::copy(flatIntrinsics.begin(), flatIntrinsics.end(), intrinsics.begin());

    for (size_t i = 0; i < 8; i++)
    {
        double distCoeff = static_cast<double>(distCoeffs[i]);
        distortions.push_back(distCoeff);
        distCoeffsDouble.push_back(distCoeff);
    }
    cameraData.distortion_model = "rational_polynomial";

    return cameraData;
}
void DepthAiOak1W::resizeCameraInfo(sensor_msgs::CameraInfo &cameraInfoMsg, int destinationWidth, int destinationHeight)
{
    float scale_y = (float)destinationHeight / cameraInfoMsg.height;
    float scale_x = (float)destinationWidth / cameraInfoMsg.width;
    cameraInfoMsg.height = destinationHeight;
    cameraInfoMsg.width = destinationWidth;

    cameraInfoMsg.K[0] *= scale_x; // fx
    cameraInfoMsg.K[2] *= scale_x; // cx
    cameraInfoMsg.K[4] *= scale_y; // fy
    cameraInfoMsg.K[5] *= scale_y; // cy

    cameraInfoMsg.P[0] *= scale_x; // fx
    cameraInfoMsg.P[2] *= scale_x; // cx
    cameraInfoMsg.P[3] *= scale_x; // T
    cameraInfoMsg.P[5] *= scale_y; // fy
    cameraInfoMsg.P[6] *= scale_y; // cy

    cameraInfoMsg.roi.x_offset = (unsigned int)(cameraInfoMsg.roi.x_offset * scale_x);
    cameraInfoMsg.roi.y_offset = (unsigned int)(cameraInfoMsg.roi.y_offset * scale_y);
    cameraInfoMsg.roi.width = (unsigned int)(cameraInfoMsg.roi.width * scale_x);
    cameraInfoMsg.roi.height = (unsigned int)(cameraInfoMsg.roi.height * scale_y);
}

int DepthAiOak1W::run()
{
    // Create pipeline
    dai::Pipeline pipeline;

    // Define source and output
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutVideo = pipeline.create<dai::node::XLinkOut>();

    xoutVideo->setStreamName("video");

    // Properties
    dai::CameraBoardSocket cameraId = dai::CameraBoardSocket::CAM_A;
    camRgb->setBoardSocket(cameraId);
    camRgb->setResolution(_resolution);
    camRgb->setFps(_fps);

    xoutVideo->input.setBlocking(false);
    xoutVideo->input.setQueueSize(1);

    // Linking
    camRgb->video.link(xoutVideo->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    auto video = device.getOutputQueue("video");

    // Get camera intrinsics
    auto calibHandler = device.readCalibration();

    int width, height;
    std::tie(std::ignore, width, height) = calibHandler.getDefaultIntrinsics(cameraId);

    std::vector<std::vector<float>> camIntrinsics;
    std::vector<float> distCoeffs;
    camIntrinsics = calibHandler.getCameraIntrinsics(cameraId, width, height, dai::Point2f(), dai::Point2f());
    distCoeffs = calibHandler.getDistortionCoefficients(cameraId);
    sensor_msgs::CameraInfo cameraData = createCameraInfo(width, height, camIntrinsics, distCoeffs);
    resizeCameraInfo(cameraData, _width, _height);

    cv::Mat camMatrix(3, 3, CV_32F);
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            camMatrix.at<float>(i, j) = cameraData.K[i * 3 + j];
        }
    }

    cv::Size size(_width, _height);
    cv::Mat newMatrix = cv::getOptimalNewCameraMatrix(
        camMatrix,
        cv::Mat(distCoeffs),
        size,
        _alpha,
        size);
    newMatrix.at<double>(2, 2) = 1.0;

    cv::Mat mapX, mapY;
    cv::initUndistortRectifyMap(
        camMatrix,
        cv::Mat(distCoeffs),
        cv::Mat(),
        newMatrix,
        size,
        CV_32FC1,
        mapX,
        mapY);

    std::vector<std::vector<float>> newCamIntrinsics;
    for (int i = 0; i < 3; i++)
    {
        std::vector<float> row;
        for (int j = 0; j < 3; j++)
        {
            row.push_back(newMatrix.at<double>(i, j));
        }
        newCamIntrinsics.push_back(row);
    }
    std::vector<float> newDistCoeffs;
    newDistCoeffs.resize(8);
    std::fill(newDistCoeffs.begin(), newDistCoeffs.end(), 0);
    sensor_msgs::CameraInfo newCameraData = createCameraInfo(_width, _height, newCamIntrinsics, newDistCoeffs);

    ros::Time rosBaseTime = ros::Time::now();
    std::chrono::time_point<std::chrono::steady_clock> steadyBaseTime = std::chrono::steady_clock::now();
    updateBaseTime(steadyBaseTime, rosBaseTime);

    while (ros::ok())
    {
        auto videoIn = video->get<dai::ImgFrame>();
        newCameraData.header.stamp = getFrameTime(rosBaseTime, steadyBaseTime, videoIn->getTimestamp());
        cameraData.header.stamp = newCameraData.header.stamp;
        cv::Mat frame = videoIn->getCvFrame();
        sensor_msgs::Image img_msg;

        _img_bridge = cv_bridge::CvImage(cameraData.header, sensor_msgs::image_encodings::BGR8, frame);
        _img_bridge.toImageMsg(img_msg);
        _camera_pub.publish(img_msg, cameraData, cameraData.header.stamp);

        // if (_rect_image_pub.getNumSubscribers() > 0)
        // {
        //     cv::Mat frameRect;
        //     cv::remap(frame, frameRect, mapX, mapY, cv::INTER_LINEAR);
        //     _img_bridge = cv_bridge::CvImage(cameraData.header, sensor_msgs::image_encodings::BGR8, frameRect);
        //     _img_bridge.toImageMsg(img_msg);
        //     _rect_image_pub.publish(img_msg);
        // }

        ros::spinOnce();
    }
    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "depthai_oak_1_w");
    ros::NodeHandle nh;
    DepthAiOak1W node(&nh);
    return node.run();
}
