#pragma once
#include <ros/ros.h>
#include <XmlRpcValue.h>

namespace bw_shared_config
{
    struct TagConfig
    {
        int tag_id;
        double tag_size;
        double x;     // meters
        double y;     // meters
        double z;     // meters
        double roll;  // degrees
        double pitch; // degrees
        double yaw;   // degrees

        static TagConfig fromXmlRpc(const XmlRpc::XmlRpcValue &data)
        {
            TagConfig config;
            config.tag_id = static_cast<int>(data["tag_id"]);
            config.tag_size = static_cast<double>(data["tag_size"]);
            config.x = static_cast<double>(data["x"]);
            config.y = static_cast<double>(data["y"]);
            config.z = static_cast<double>(data["z"]);
            config.roll = static_cast<double>(data["roll"]);
            config.pitch = static_cast<double>(data["pitch"]);
            config.yaw = static_cast<double>(data["yaw"]);
            return config;
        }
    };
}