#include "field_estimation.h"


FieldEstimation::FieldEstimation(ros::NodeHandle* nodehandle) :
    BaseEstimation(nodehandle)
{
    double ransac_threshold;
    ros::param::param<double>("~ransac_threshold", ransac_threshold, 0.01);
    int ransac_max_iterations;
    ros::param::param<int>("~ransac_max_iterations", ransac_max_iterations, 100);

    _estimator = new GRANSAC::RANSAC<PlaneModel, 3>();
    _estimator->Initialize(ransac_threshold, ransac_max_iterations);

    _field_pub = nh.advertise<bw_interfaces::EstimatedObject>("estimation/field", _queue_size);
    _field_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("estimation/field_pose", _queue_size);

    ROS_INFO("FieldEstimation node initialized");
}

FieldEstimation::~FieldEstimation()
{
}

void FieldEstimation::synced_callback(
    const sensor_msgs::ImageConstPtr& depth_image,
    const bw_interfaces::SegmentationInstanceArrayConstPtr& segmentation)
{
    cv::Mat depth_cv_image;
    if (!get_depth_image(depth_cv_image, depth_image)) {
        return;
    }

    std::vector<std::vector<cv::Point>> field_contours;
    cv::Mat field_mask = cv::Mat::zeros(depth_cv_image.rows, depth_cv_image.cols, CV_8UC1);
    for (size_t index = 0; index < segmentation->instances.size(); index++)
    {
        bw_interfaces::SegmentationInstance instance = segmentation->instances[index];
        if (is_label_included(instance.label)) {
            std::vector<std::vector<cv::Point>> cv_contours = get_cv_contours(instance.contours);
            cv::drawContours(field_mask, cv_contours, -1, cv::Scalar(255), cv::FILLED);
            for (size_t contour_index = 0; contour_index < cv_contours.size(); contour_index++)
            {
                field_contours.push_back(cv_contours[contour_index]);
            }
        }
    }

    geometry_msgs::PoseStamped field_pose;
    field_pose.header = segmentation->header;

    for (size_t index = 0; index < segmentation->instances.size(); index++)
    {
        bw_interfaces::SegmentationInstance instance = segmentation->instances[index];
        if (is_label_included(instance.label)) {
            continue;
        }

        std::vector<std::vector<cv::Point>> cv_contours = get_cv_contours(instance.contours);
        cv::drawContours(field_mask, cv_contours, -1, cv::Scalar(0), cv::FILLED);
    }

    if (field_contours.size() > 0) {
        bw_interfaces::EstimatedObject field_msg = find_plane(depth_cv_image, field_mask, field_contours);
        field_msg.header = segmentation->header;
        _field_pub.publish(field_msg);
        field_pose.pose = field_msg.state.pose.pose;
        _field_pose_pub.publish(field_pose);
    }
}

bw_interfaces::EstimatedObject FieldEstimation::find_plane(cv::Mat depth_image, cv::Mat mask, std::vector<std::vector<cv::Point>> cv_contours) 
{
    std::vector<Vector3VP> point_cloud;

    for (size_t x = 0; x < depth_image.cols; x++)
    {
        for (size_t y = 0; y < depth_image.rows; y++)
        {
            size_t index = (y * depth_image.cols + x) * 3;
            int mask_value = mask.at<int>(y, x);
            if (mask_value == 0) {
                
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
    plane_fitting(point_cloud, center, coefs);

    ROS_INFO("coeffs: %f, %f, %f, %f", coefs[0], coefs[1], coefs[2], coefs[3]);

    bw_interfaces::EstimatedObject field_msg;

    field_msg.state.pose.pose.position.x = center[0];
    field_msg.state.pose.pose.position.y = center[1];
    field_msg.state.pose.pose.position.z = center[2];

    Eigen::Vector3d normal(coefs[0], coefs[1], coefs[2]);
    Eigen::Vector3d up(0, 1, 0); // define up direction
    Eigen::Vector3d axis = normal.cross(up); // get axis of rotation
    double angle = acos(normal.dot(up)); // get angle of rotation
    Eigen::Quaterniond q(Eigen::AngleAxisd(angle, axis)); // create quaternion

    // Ensure that the quaternion is normalized
    q.normalize();

    field_msg.state.pose.pose.orientation.x = q.x();
    field_msg.state.pose.pose.orientation.y = q.y();
    field_msg.state.pose.pose.orientation.z = q.z();
    field_msg.state.pose.pose.orientation.w = q.w();

    return field_msg;
}

bool FieldEstimation::plane_fitting(const std::vector<Vector3VP> &points_input, double* center, double* normal)
{
    int Num = points_input.size();
    std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> CandPoints;
    CandPoints.resize(Num);
#pragma omp parallel for
    for (int i = 0; i < Num; ++i)
    {
        Vector3VP p = points_input[i];
        std::shared_ptr<GRANSAC::AbstractParameter> CandPt = std::make_shared<Point3D>(p[0], p[1], p[2]);
        CandPoints[i]=CandPt;
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
