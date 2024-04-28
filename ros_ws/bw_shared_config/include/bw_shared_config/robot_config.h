#pragma once
#include <ros/ros.h>
#include <XmlRpcValue.h>

namespace bw_shared_config
{
    enum class RobotTeam
    {
        OUR_TEAM,
        THEIR_TEAM,
        REFEREE
    };

    struct RobotConfig
    {
        std::string name;
        int up_id = -1;
        int down_id = -1;
        double radius = 0.0;
        int bridge_id = -1;
        double base_width = 1.0;
        double height = 0.025;

        RobotTeam team() const
        {
            if (name.find("referee") != std::string::npos)
            {
                return RobotTeam::REFEREE;
            }
            else
            {
                return (up_id >= 0 || down_id >= 0) ? RobotTeam::OUR_TEAM : RobotTeam::THEIR_TEAM;
            }
        }

        static RobotConfig fromXmlRpc(const XmlRpc::XmlRpcValue &data)
        {
            RobotConfig config;
            config.name = static_cast<std::string>(data["name"]);
            config.up_id = static_cast<int>(data["up_id"]);
            config.down_id = static_cast<int>(data["down_id"]);
            config.radius = static_cast<double>(data["radius"]);
            config.bridge_id = static_cast<int>(data["bridge_id"]);
            config.base_width = static_cast<double>(data["base_width"]);
            config.height = static_cast<double>(data["height"]);
            return config;
        }
    };
}
