//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.BwInterfaces
{
    [Serializable]
    public class ContourMsg : Message
    {
        public const string k_RosMessageName = "bw_interfaces/Contour";
        public override string RosMessageName => k_RosMessageName;

        public UVKeypointMsg[] points;

        public ContourMsg()
        {
            this.points = new UVKeypointMsg[0];
        }

        public ContourMsg(UVKeypointMsg[] points)
        {
            this.points = points;
        }

        public static ContourMsg Deserialize(MessageDeserializer deserializer) => new ContourMsg(deserializer);

        private ContourMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.points, UVKeypointMsg.Deserialize, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.WriteLength(this.points);
            serializer.Write(this.points);
        }

        public override string ToString()
        {
            return "ContourMsg: " +
            "\npoints: " + System.String.Join(", ", points.ToList());
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