//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.BwInterfaces
{
    [Serializable]
    public class HealthSummaryMsg : Message
    {
        public const string k_RosMessageName = "bw_interfaces/HealthSummary";
        public override string RosMessageName => k_RosMessageName;

        public Std.HeaderMsg header;
        public string[] active_nodes;
        public string[] dead_nodes;

        public HealthSummaryMsg()
        {
            this.header = new Std.HeaderMsg();
            this.active_nodes = new string[0];
            this.dead_nodes = new string[0];
        }

        public HealthSummaryMsg(Std.HeaderMsg header, string[] active_nodes, string[] dead_nodes)
        {
            this.header = header;
            this.active_nodes = active_nodes;
            this.dead_nodes = dead_nodes;
        }

        public static HealthSummaryMsg Deserialize(MessageDeserializer deserializer) => new HealthSummaryMsg(deserializer);

        private HealthSummaryMsg(MessageDeserializer deserializer)
        {
            this.header = Std.HeaderMsg.Deserialize(deserializer);
            deserializer.Read(out this.active_nodes, deserializer.ReadLength());
            deserializer.Read(out this.dead_nodes, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.WriteLength(this.active_nodes);
            serializer.Write(this.active_nodes);
            serializer.WriteLength(this.dead_nodes);
            serializer.Write(this.dead_nodes);
        }

        public override string ToString()
        {
            return "HealthSummaryMsg: " +
            "\nheader: " + header.ToString() +
            "\nactive_nodes: " + System.String.Join(", ", active_nodes.ToList()) +
            "\ndead_nodes: " + System.String.Join(", ", dead_nodes.ToList());
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