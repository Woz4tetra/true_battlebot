#include <bw_shared_config/bw_shared_config.h>

namespace bw_shared_config
{
    SharedConfig getSharedConfig()
    {
        ros::NodeHandle nh;
        std::string key = "/shared_config";
        while (!nh.hasParam(key))
        {
            ROS_INFO_STREAM_THROTTLE(3, "Waiting for " << key << " to be set");
            ros::Duration(0.1).sleep();
        }
        XmlRpc::XmlRpcValue shared_config;
        nh.getParam(key, shared_config);

        return SharedConfig::fromXmlRpc(shared_config);
    }
}