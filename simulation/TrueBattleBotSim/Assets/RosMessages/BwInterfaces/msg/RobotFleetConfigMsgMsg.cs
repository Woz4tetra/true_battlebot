//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.BwInterfaces
{
    [Serializable]
    public class RobotFleetConfigMsgMsg : Message
    {
        public const string k_RosMessageName = "bw_interfaces/RobotFleetConfigMsg";
        public override string RosMessageName => k_RosMessageName;

        public RobotConfigMsgMsg[] robots;

        public RobotFleetConfigMsgMsg()
        {
            this.robots = new RobotConfigMsgMsg[0];
        }

        public RobotFleetConfigMsgMsg(RobotConfigMsgMsg[] robots)
        {
            this.robots = robots;
        }

        public static RobotFleetConfigMsgMsg Deserialize(MessageDeserializer deserializer) => new RobotFleetConfigMsgMsg(deserializer);

        private RobotFleetConfigMsgMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.robots, RobotConfigMsgMsg.Deserialize, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.WriteLength(this.robots);
            serializer.Write(this.robots);
        }

        public override string ToString()
        {
            return "RobotFleetConfigMsgMsg: " +
            "\nrobots: " + System.String.Join(", ", robots.ToList());
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
