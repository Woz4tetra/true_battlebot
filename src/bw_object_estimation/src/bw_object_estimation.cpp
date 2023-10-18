#include "bw_object_estimation.h"


ObjectEstimation::ObjectEstimation(ros::NodeHandle* nodehandle) :
    nh(*nodehandle)
{
}

ObjectEstimation::~ObjectEstimation()
{
}

int ObjectEstimation::run()
{
    ros::spin();

    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_estimation");
    ros::NodeHandle nh;
    ObjectEstimation node(&nh);
    return node.run();
}
