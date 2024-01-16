#include "field_estimation.h"

FieldEstimation::FieldEstimation(ros::NodeHandle *nodehandle) : nh(*nodehandle)
{
    ros::param::param<std::vector<std::string>>("~include_labels", _include_labels, std::vector<std::string>());
    if (_include_labels.size() == 0)
    {
        ROS_INFO("No labels specified, including all labels");
    }
    double ransac_threshold;
    ros::param::param<double>("~ransac_threshold", ransac_threshold, 0.01);
    int ransac_max_iterations;
    ros::param::param<int>("~ransac_max_iterations", ransac_max_iterations, 100);

    _estimator = new GRANSAC::RANSAC<PlaneModel, 3>();
    _estimator->Initialize(ransac_threshold, ransac_max_iterations);

    _depth_info_sub = nh.subscribe<sensor_msgs::CameraInfo>("depth/camera_info", 1, &FieldEstimation::camera_info_callback, this);
    _request_sub = nh.subscribe<geometry_msgs::PointStamped>("plane_request", 1, &FieldEstimation::plane_request_callback, this);

    _response_pub = nh.advertise<geometry_msgs::PoseStamped>("plane_response", 1);

    ROS_INFO("FieldEstimation node initialized");
}

FieldEstimation::~FieldEstimation()
{
}

void FieldEstimation::camera_info_callback(const sensor_msgs::CameraInfoConstPtr &camera_info)
{
    _camera_model.fromCameraInfo(camera_info);
    _depth_info_sub.shutdown();
    ROS_INFO("Camera model loaded");
}

void FieldEstimation::plane_request_callback(const geometry_msgs::PointStampedConstPtr &request)
{
    sensor_msgs::ImageConstPtr depth_image =
        ros::topic::waitForMessage<sensor_msgs::Image>("depth/image_raw", nh, ros::Duration(1));
    if (depth_image == NULL)
    {
        ROS_WARN("No depth image received");
        return;
    }
    bw_interfaces::SegmentationInstanceArrayConstPtr segmentation =
        ros::topic::waitForMessage<bw_interfaces::SegmentationInstanceArray>("segmentation", nh, ros::Duration(1));
    cv::Mat depth_cv_image;
    if (!get_depth_image(_camera_model, depth_cv_image, depth_image))
    {
        return;
    }

    int num_contours = 0;
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
        if (!is_label_included(instance.label))
        {
            std::vector<std::vector<cv::Point>> cv_contours = get_cv_contours(instance.contours);
            cv::drawContours(field_mask, cv_contours, -1, cv::Scalar(0), cv::FILLED);
        }
    }

    if (num_contours > 0)
    {
        geometry_msgs::PoseStamped response_msg;
        if (find_plane(segmentation->header, depth_cv_image, field_mask, response_msg))
        {
            _response_pub.publish(response_msg);
        }
    }
}

bool FieldEstimation::is_label_included(std::string label)
{
    if (_include_labels.size() == 0)
    {
        return true;
    }
    else
    {
        for (size_t index = 0; index < _include_labels.size(); index++)
        {
            if (label.compare(_include_labels[index]) == 0)
            {
                return true;
            }
        }
        return false;
    }
}

bool FieldEstimation::find_plane(std_msgs::Header header, cv::Mat depth_image, cv::Mat mask, geometry_msgs::PoseStamped &pose_msg)
{
    std::vector<Vector3VP> point_cloud;

    for (size_t x = 0; x < depth_image.cols; x++)
    {
        for (size_t y = 0; y < depth_image.rows; y++)
        {
            int mask_value = mask.at<int>(y, x);
            if (mask_value == 0)
            {
                continue;
            }
            float z_meters = depth_image.at<float>(y, x);
            cv::Point3d ray = _camera_model.projectPixelTo3dRay(cv::Point2d(x, y));
            float x_meters = ray.x * z_meters;
            float y_meters = ray.y * z_meters;

            Vector3VP point_3d = {x_meters, y_meters, z_meters};
            point_cloud.push_back(point_3d);
        }
    }

    double *center = new double[3];
    double *coefs = new double[4];
    if (!plane_fitting(point_cloud, center, coefs))
    {
        ROS_WARN("Plane fitting failed");
        return false;
    }

    ROS_INFO("coeffs: %f, %f, %f, %f", coefs[0], coefs[1], coefs[2], coefs[3]);

    Eigen::Vector3d normal(coefs[0], coefs[1], coefs[2]);
    Eigen::Vector3d up(1, 0, 0);                          // define up direction
    Eigen::Vector3d axis = normal.cross(up);              // get axis of rotation
    double angle = acos(normal.dot(up));                  // get angle of rotation
    Eigen::Quaterniond q(Eigen::AngleAxisd(angle, axis)); // create quaternion

    normal.normalize();

    // Ensure that the quaternion is normalized
    q.normalize();

    pose_msg.pose.position.x = center[0];
    pose_msg.pose.position.y = center[1];
    pose_msg.pose.position.z = center[2];

    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    pose_msg.header = header;

    return true;
}

bool FieldEstimation::plane_fitting(const std::vector<Vector3VP> &points_input, double *center, double *normal)
{
    int Num = points_input.size();
    std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> CandPoints;
    CandPoints.resize(Num);
#pragma omp parallel for
    for (int i = 0; i < Num; ++i)
    {
        Vector3VP p = points_input[i];
        std::shared_ptr<GRANSAC::AbstractParameter> CandPt = std::make_shared<Point3D>(p[0], p[1], p[2]);
        CandPoints[i] = CandPt;
    }

    int64_t start = cv::getTickCount();
    _estimator->Estimate(CandPoints);
    int64_t end = cv::getTickCount();
    std::cout << "RANSAC took: " << GRANSAC::VPFloat(end - start) / GRANSAC::VPFloat(cv::getTickFrequency()) * 1000.0 << " ms." << std::endl;

    auto BestPlane = _estimator->GetBestModel();
    if (BestPlane == nullptr)
    {
        return false;
    }
    for (int i = 0; i < 3; i++)
    {
        center[i] = BestPlane->m_PointCenter[i];
    }
    for (int i = 0; i < 4; i++)
    {
        normal[i] = BestPlane->m_PlaneCoefs[i];
    }

    return true;
}

int FieldEstimation::run()
{
    ros::spin();

    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "field_estimation");
    ros::NodeHandle nh;
    FieldEstimation node(&nh);
    return node.run();
}
