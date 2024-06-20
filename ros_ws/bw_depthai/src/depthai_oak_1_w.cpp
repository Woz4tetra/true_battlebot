#include "depthai_oak_1_w.h"

DepthAiOak1W::DepthAiOak1W(ros::NodeHandle *nodehandle) : nh(*nodehandle),
                                                          _image_transport(nh)
{
    std::string resolution_name;
    ros::param::param<std::string>("~resolution", resolution_name, "1080_P");
    ros::param::param<std::string>("~camera_name", _camera_name, "oak");

    if (resolution_name.compare("1080_P") == 0)
    {
        _resolution = dai::ColorCameraProperties::SensorResolution::THE_1080_P;
    }
    else if (resolution_name.compare("4_K") == 0)
    {
        _resolution = dai::ColorCameraProperties::SensorResolution::THE_4_K;
    }
    else if (resolution_name.compare("12_MP") == 0)
    {
        _resolution = dai::ColorCameraProperties::SensorResolution::THE_12_MP;
    }
    else if (resolution_name.compare("13_MP") == 0)
    {
        _resolution = dai::ColorCameraProperties::SensorResolution::THE_13_MP;
    }
    else if (resolution_name.compare("720_P") == 0)
    {
        _resolution = dai::ColorCameraProperties::SensorResolution::THE_720_P;
    }
    else if (resolution_name.compare("800_P") == 0)
    {
        _resolution = dai::ColorCameraProperties::SensorResolution::THE_800_P;
    }
    else if (resolution_name.compare("1200_P") == 0)
    {
        _resolution = dai::ColorCameraProperties::SensorResolution::THE_1200_P;
    }
    else if (resolution_name.compare("5_MP") == 0)
    {
        _resolution = dai::ColorCameraProperties::SensorResolution::THE_5_MP;
    }
    else if (resolution_name.compare("4000X3000") == 0)
    {
        _resolution = dai::ColorCameraProperties::SensorResolution::THE_4000X3000;
    }
    else if (resolution_name.compare("5312X6000") == 0)
    {
        _resolution = dai::ColorCameraProperties::SensorResolution::THE_5312X6000;
    }
    else if (resolution_name.compare("48_MP") == 0)
    {
        _resolution = dai::ColorCameraProperties::SensorResolution::THE_48_MP;
    }
    else if (resolution_name.compare("1440X1080") == 0)
    {
        _resolution = dai::ColorCameraProperties::SensorResolution::THE_1440X1080;
    }
    else if (resolution_name.compare("1352X1012") == 0)
    {
        _resolution = dai::ColorCameraProperties::SensorResolution::THE_1352X1012;
    }
    else if (resolution_name.compare("2024X1520") == 0)
    {
        _resolution = dai::ColorCameraProperties::SensorResolution::THE_2024X1520;
    }
    else
    {
        ROS_ERROR("Invalid resolution name: %s", resolution_name.c_str());
        ros::shutdown();
    }

    _camera_pub = _image_transport.advertiseCamera(_camera_name, 10);
}

DepthAiOak1W::~DepthAiOak1W()
{
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

    xoutVideo->input.setBlocking(false);
    xoutVideo->input.setQueueSize(1);

    // Linking
    camRgb->video.link(xoutVideo->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    auto video = device.getOutputQueue("video");

    // Get camera intrinsics
    auto calibHandler = device.readCalibration();

    sensor_msgs::CameraInfo cameraData;
    int width, height;
    std::tie(std::ignore, width, height) = calibHandler.getDefaultIntrinsics(cameraId);
    cameraData.width = width;
    cameraData.height = height;
    cameraData.header.frame_id = _camera_name;

    std::vector<std::vector<float>> camIntrinsics, rectifiedRotation;
    std::vector<float> distCoeffs;
    std::vector<double> flatIntrinsics, distCoeffsDouble;
    camIntrinsics = calibHandler.getCameraIntrinsics(cameraId, cameraData.width, cameraData.height, cv::Point2f(), cv::Point2f());

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

    distCoeffs = calibHandler.getDistortionCoefficients(cameraId);

    for (size_t i = 0; i < 8; i++)
    {
        distortions.push_back(static_cast<double>(distCoeffs[i]));
    }
    cameraData.distortion_model = "rational_polynomial";

    while (ros::ok())
    {
        auto videoIn = video->get<dai::ImgFrame>();
        cv::Mat frame = videoIn->getCvFrame();
        uint64_t n_sec = std::chrono::duration_cast<std::chrono::nanoseconds>(videoIn->getTimestampDevice()).count();
        cameraData.header.stamp = ros::Time(0).fromNSec(nsec);
        _img_bridge = cv_bridge::CvImage(cameraData.header, sensor_msgs::image_encodings::RGB8, frame);
        sensor_msgs::Image img_msg;
        _img_bridge.toImageMsg(img_msg);
        _camera_pub.publish(img_msg, cameraData, cameraData.header.stamp);

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
