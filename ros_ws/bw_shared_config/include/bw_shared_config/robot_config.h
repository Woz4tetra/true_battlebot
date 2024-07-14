#pragma once
#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <bw_shared_config/tag_config.h>

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
        std::vector<TagConfig> tags;
        double radius = 0.0;
        bool is_controlled = false;

        static RobotConfig fromXmlRpc(const XmlRpc::XmlRpcValue &data)
        {
            RobotConfig config;
            config.name = static_cast<std::string>(data["name"]);
            config.team = RobotTeam::fromString(static_cast<std::string>(data["team"]));
            for (int i = 0; i < data["tags"].size(); ++i)
            {
                config.tags.push_back(TagConfig::fromXmlRpc(data["tags"][i]));
            }
            config.radius = static_cast<double>(data["radius"]);
            config.is_controlled = static_cast<bool>(data["is_controlled"]);
            return config;
        }
    };
}
