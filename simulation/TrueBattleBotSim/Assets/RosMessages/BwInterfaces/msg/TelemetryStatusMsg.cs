//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.BwInterfaces
{
    [Serializable]
    public class TelemetryStatusMsg : Message
    {
        public const string k_RosMessageName = "bw_interfaces/TelemetryStatus";
        public override string RosMessageName => k_RosMessageName;

        public bool controller_connected;
        public bool is_armed;
        public bool is_ready;
        public bool is_connected;
        public float battery_voltage;
        public float battery_current;
        public float battery_consumption;
        public string link_stats_json;
        public string flight_mode;

        public TelemetryStatusMsg()
        {
            this.controller_connected = false;
            this.is_armed = false;
            this.is_ready = false;
            this.is_connected = false;
            this.battery_voltage = 0.0f;
            this.battery_current = 0.0f;
            this.battery_consumption = 0.0f;
            this.link_stats_json = "";
            this.flight_mode = "";
        }

        public TelemetryStatusMsg(bool controller_connected, bool is_armed, bool is_ready, bool is_connected, float battery_voltage, float battery_current, float battery_consumption, string link_stats_json, string flight_mode)
        {
            this.controller_connected = controller_connected;
            this.is_armed = is_armed;
            this.is_ready = is_ready;
            this.is_connected = is_connected;
            this.battery_voltage = battery_voltage;
            this.battery_current = battery_current;
            this.battery_consumption = battery_consumption;
            this.link_stats_json = link_stats_json;
            this.flight_mode = flight_mode;
        }

        public static TelemetryStatusMsg Deserialize(MessageDeserializer deserializer) => new TelemetryStatusMsg(deserializer);

        private TelemetryStatusMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.controller_connected);
            deserializer.Read(out this.is_armed);
            deserializer.Read(out this.is_ready);
            deserializer.Read(out this.is_connected);
            deserializer.Read(out this.battery_voltage);
            deserializer.Read(out this.battery_current);
            deserializer.Read(out this.battery_consumption);
            deserializer.Read(out this.link_stats_json);
            deserializer.Read(out this.flight_mode);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.controller_connected);
            serializer.Write(this.is_armed);
            serializer.Write(this.is_ready);
            serializer.Write(this.is_connected);
            serializer.Write(this.battery_voltage);
            serializer.Write(this.battery_current);
            serializer.Write(this.battery_consumption);
            serializer.Write(this.link_stats_json);
            serializer.Write(this.flight_mode);
        }

        public override string ToString()
        {
            return "TelemetryStatusMsg: " +
            "\ncontroller_connected: " + controller_connected.ToString() +
            "\nis_armed: " + is_armed.ToString() +
            "\nis_ready: " + is_ready.ToString() +
            "\nis_connected: " + is_connected.ToString() +
            "\nbattery_voltage: " + battery_voltage.ToString() +
            "\nbattery_current: " + battery_current.ToString() +
            "\nbattery_consumption: " + battery_consumption.ToString() +
            "\nlink_stats_json: " + link_stats_json.ToString() +
            "\nflight_mode: " + flight_mode.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
