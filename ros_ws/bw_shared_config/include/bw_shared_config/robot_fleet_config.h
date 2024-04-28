#pragma once
#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <bw_shared_config/robot_config.h>

namespace bw_shared_config
{
    struct RobotFleetConfig
    {
        std::vector<RobotConfig> robots;

        static RobotFleetConfig fromXmlRpc(const XmlRpc::XmlRpcValue &data)
        {
            RobotFleetConfig config;
            XmlRpc::XmlRpcValue robot_param = data["robots"];
            for (int i = 0; i < robot_param.size(); ++i)
            {
                config.robots.push_back(RobotConfig::fromXmlRpc(robot_param[i]));
            }
            return config;
        }
    };
}
