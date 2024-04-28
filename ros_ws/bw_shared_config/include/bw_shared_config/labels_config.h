#pragma once
#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <map>
#include <stdexcept>
#include <bw_shared_config/label.h>

namespace bw_shared_config
{
    struct LabelsConfig
    {
        std::map<Label::Value, LabelConfig> labels;

        static LabelsConfig fromXmlRpc(const XmlRpc::XmlRpcValue &data)
        {
            LabelsConfig config;
            XmlRpc::XmlRpcValue label_param = data["labels"];
            for (int i = 0; i < label_param.size(); ++i)
            {
                LabelConfig labelConfig = LabelConfig::fromXmlRpc(label_param[i]);
                if (config.labels.count(labelConfig.type) != 0)
                {
                    throw std::runtime_error("Duplicate label: " + labelConfig.name);
                }
                config.labels[labelConfig.type] = labelConfig;
            }
            return config;
        }

        bool hasKey(std::string key)
        {
            return labels.count(Label::fromString(key)) != 0;
        }

        LabelConfig get(std::string key)
        {
            return labels[Label::fromString(key)];
        }
    };
}