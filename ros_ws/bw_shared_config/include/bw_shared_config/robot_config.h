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
        RobotTeam team;
        std::vector<int> ids;
        double radius = 0.0;

        static RobotConfig fromXmlRpc(const XmlRpc::XmlRpcValue &data)
        {
            RobotConfig config;
            config.name = static_cast<std::string>(data["name"]);
            config.team = static_cast<RobotTeam>(static_cast<int>(data["team"]));
            for (int i = 0; i < data["ids"].size(); ++i)
            {
                config.ids.push_back(static_cast<int>(data["ids"][i]));
            }
            config.radius = static_cast<double>(data["radius"]);
            return config;
        }
    };
}
