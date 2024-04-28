#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <vector>
#include <string>

namespace bw_shared_config
{
    struct Label
    {
        enum Value
        {
            ROBOT,
            FIELD,
            REFEREE,
            FRIENDLY_ROBOT,
            CONTROLLED_ROBOT
        };

        static Value fromString(const std::string &str)
        {
            if (str == "robot")
            {
                return ROBOT;
            }
            else if (str == "field")
            {
                return FIELD;
            }
            else if (str == "referee")
            {
                return REFEREE;
            }
            else if (str == "friendly_robot")
            {
                return FRIENDLY_ROBOT;
            }
            else if (str == "controlled_robot")
            {
                return CONTROLLED_ROBOT;
            }
            else
            {
                throw std::runtime_error("Invalid Label: " + str);
            }
        }
    };
    struct LabelConfig
    {
        std::string name;
        float height;
        Label::Value type;

        static LabelConfig fromXmlRpc(const XmlRpc::XmlRpcValue &data)
        {
            LabelConfig config;
            config.name = static_cast<std::string>(data["name"]);
            config.height = static_cast<double>(data["height"]);
            config.type = Label::fromString(config.name);
            return config;
        }
    };
}