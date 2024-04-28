#pragma once
#include <ros/ros.h>
#include <XmlRpcValue.h>

namespace bw_shared_config
{
    struct Size
    {
        double x;
        double y;
        double z;

        static Size fromXmlRpc(const XmlRpc::XmlRpcValue &data)
        {
            Size size;
            size.x = static_cast<double>(data["x"]);
            size.y = static_cast<double>(data["y"]);
            size.z = static_cast<double>(data["z"]);
            return size;
        }
    };
}
