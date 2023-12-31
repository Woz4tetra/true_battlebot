//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.ZedInterfaces
{
    [Serializable]
    public class BoundingBox2DfMsg : Message
    {
        public const string k_RosMessageName = "zed_interfaces/BoundingBox2Df";
        public override string RosMessageName => k_RosMessageName;

        //       0 ------- 1
        //       |         |
        //       |         |
        //       |         |
        //       3 ------- 2
        public Keypoint2DfMsg[] corners;

        public BoundingBox2DfMsg()
        {
            this.corners = new Keypoint2DfMsg[4];
        }

        public BoundingBox2DfMsg(Keypoint2DfMsg[] corners)
        {
            this.corners = corners;
        }

        public static BoundingBox2DfMsg Deserialize(MessageDeserializer deserializer) => new BoundingBox2DfMsg(deserializer);

        private BoundingBox2DfMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.corners, Keypoint2DfMsg.Deserialize, 4);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.corners);
        }

        public override string ToString()
        {
            return "BoundingBox2DfMsg: " +
            "\ncorners: " + System.String.Join(", ", corners.ToList());
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
