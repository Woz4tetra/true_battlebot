#pragma once
#include <ros/ros.h>
#include <XmlRpcValue.h>

namespace bw_shared_config
{
    struct RobotTeam
    {
        enum Value
        {
            OUR_TEAM,
            THEIR_TEAM,
            REFEREE
        };

        static Value fromString(const std::string &str)
        {
            if (str == "our_team")
            {
                return OUR_TEAM;
            }
            else if (str == "their_team")
            {
                return THEIR_TEAM;
            }
            else if (str == "referee")
            {
                return REFEREE;
            }
            else
            {
                throw std::runtime_error("Invalid RobotTeam: " + str);
            }
        }
    };

    struct RobotConfig
    {
        std::string name;
        RobotTeam::Value team;
        std::vector<int> ids;
        double radius = 0.0;

        static RobotConfig fromXmlRpc(const XmlRpc::XmlRpcValue &data)
        {
            RobotConfig config;
            config.name = static_cast<std::string>(data["name"]);
            config.team = RobotTeam::fromString(static_cast<std::string>(data["team"]));
            for (int i = 0; i < data["ids"].size(); ++i)
            {
                config.ids.push_back(static_cast<int>(data["ids"][i]));
            }
            config.radius = static_cast<double>(data["radius"]);
            return config;
        }
    };
}
