#pragma once
#include <ros/ros.h>
#include <string.h>
#include <XmlRpcValue.h>
#include <bw_shared_config/size.h>

namespace bw_shared_config
{
    struct FieldType
    {
        enum Value
        {
            NHRL_SMALL,
            NHRL_LARGE,
            MEATBALL_TESTBOX
        };

        static Value fromString(const std::string &str)
        {
            if (str == "nhrl_small")
            {
                return NHRL_SMALL;
            }
            else if (str == "nhrl_large")
            {
                return NHRL_LARGE;
            }
            else if (str == "meatball_testbox")
            {
                return MEATBALL_TESTBOX;
            }
            else
            {
                throw std::runtime_error("Invalid FieldType: " + str);
            }
        }
    };

    struct MapConfig
    {
        std::string name;
        Size size;
        FieldType::Value type;

        static MapConfig fromXmlRpc(const XmlRpc::XmlRpcValue &data)
        {
            MapConfig config;
            config.name = static_cast<std::string>(data["name"]);
            config.size = Size::fromXmlRpc(data["size"]);
            config.type = FieldType::fromString(config.name);
            return config;
        }
    };
}
