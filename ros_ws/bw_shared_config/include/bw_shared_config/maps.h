#pragma once
#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <map>
#include <stdexcept>
#include <bw_shared_config/size.h>
#include <bw_shared_config/map_config.h>

namespace bw_shared_config
{
    struct Maps
    {
        std::map<FieldType::Value, MapConfig> maps;

        static Maps fromXmlRpc(const XmlRpc::XmlRpcValue &data)
        {
            Maps maps;
            XmlRpc::XmlRpcValue maps_param = data["maps"];
            for (int i = 0; i < maps_param.size(); ++i)
            {
                MapConfig mapConfig = MapConfig::fromXmlRpc(maps_param[i]);
                if (maps.maps.count(mapConfig.type) != 0)
                {
                    throw std::runtime_error("Duplicate map: " + mapConfig.name);
                }
                maps.maps[mapConfig.type] = mapConfig;
            }
            return maps;
        }

        bool hasKey(std::string key)
        {
            return maps.count(FieldType::fromString(key)) != 0;
        }

        MapConfig get(std::string key)
        {
            return maps[FieldType::fromString(key)];
        }
    };
}
