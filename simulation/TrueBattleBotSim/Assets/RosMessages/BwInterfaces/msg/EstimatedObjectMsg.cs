//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.BwInterfaces
{
    [Serializable]
    public class EstimatedObjectMsg : Message
    {
        public const string k_RosMessageName = "bw_interfaces/EstimatedObject";
        public override string RosMessageName => k_RosMessageName;

        public Std.HeaderMsg header;
        public Nav.OdometryMsg state;
        public Geometry.Vector3Msg size;
        public string label;

        public EstimatedObjectMsg()
        {
            this.header = new Std.HeaderMsg();
            this.state = new Nav.OdometryMsg();
            this.size = new Geometry.Vector3Msg();
            this.label = "";
        }

        public EstimatedObjectMsg(Std.HeaderMsg header, Nav.OdometryMsg state, Geometry.Vector3Msg size, string label)
        {
            this.header = header;
            this.state = state;
            this.size = size;
            this.label = label;
        }

        public static EstimatedObjectMsg Deserialize(MessageDeserializer deserializer) => new EstimatedObjectMsg(deserializer);

        private EstimatedObjectMsg(MessageDeserializer deserializer)
        {
            this.header = Std.HeaderMsg.Deserialize(deserializer);
            this.state = Nav.OdometryMsg.Deserialize(deserializer);
            this.size = Geometry.Vector3Msg.Deserialize(deserializer);
            deserializer.Read(out this.label);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.state);
            serializer.Write(this.size);
            serializer.Write(this.label);
        }

        public override string ToString()
        {
            return "EstimatedObjectMsg: " +
            "\nheader: " + header.ToString() +
            "\nstate: " + state.ToString() +
            "\nsize: " + size.ToString() +
            "\nlabel: " + label.ToString();
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