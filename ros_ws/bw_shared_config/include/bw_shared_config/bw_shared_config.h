#pragma once
#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <bw_shared_config/maps.h>
#include <bw_shared_config/robot_fleet_config.h>
#include <bw_shared_config/labels_config.h>

namespace bw_shared_config
{
    struct SharedConfig
    {
        Maps maps;
        RobotFleetConfig robots;
        LabelsConfig labels;
        RobotFleetConfig opponent_templates;

        static SharedConfig fromXmlRpc(const XmlRpc::XmlRpcValue &data)
        {
            SharedConfig config;
            config.maps = Maps::fromXmlRpc(data["maps"]);
            config.robots = RobotFleetConfig::fromXmlRpc(data["robots"]);
            config.labels = LabelsConfig::fromXmlRpc(data["labels"]);
            config.opponent_templates = RobotFleetConfig::fromXmlRpc(data["opponent_templates"]);
            return config;
        }
    };
    SharedConfig getSharedConfig();
}
