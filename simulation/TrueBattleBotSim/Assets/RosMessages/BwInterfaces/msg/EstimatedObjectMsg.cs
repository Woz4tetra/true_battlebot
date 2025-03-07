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
        public string child_frame_id;
        public Geometry.PoseWithCovarianceMsg pose;
        public Geometry.TwistWithCovarianceMsg twist;
        public Geometry.Vector3Msg size;
        public string label;
        public Geometry.PoseMsg[] keypoints;
        public string[] keypoint_names;
        public double score;

        public EstimatedObjectMsg()
        {
            this.header = new Std.HeaderMsg();
            this.child_frame_id = "";
            this.pose = new Geometry.PoseWithCovarianceMsg();
            this.twist = new Geometry.TwistWithCovarianceMsg();
            this.size = new Geometry.Vector3Msg();
            this.label = "";
            this.keypoints = new Geometry.PoseMsg[0];
            this.keypoint_names = new string[0];
            this.score = 0.0;
        }

        public EstimatedObjectMsg(Std.HeaderMsg header, string child_frame_id, Geometry.PoseWithCovarianceMsg pose, Geometry.TwistWithCovarianceMsg twist, Geometry.Vector3Msg size, string label, Geometry.PoseMsg[] keypoints, string[] keypoint_names, double score)
        {
            this.header = header;
            this.child_frame_id = child_frame_id;
            this.pose = pose;
            this.twist = twist;
            this.size = size;
            this.label = label;
            this.keypoints = keypoints;
            this.keypoint_names = keypoint_names;
            this.score = score;
        }

        public static EstimatedObjectMsg Deserialize(MessageDeserializer deserializer) => new EstimatedObjectMsg(deserializer);

        private EstimatedObjectMsg(MessageDeserializer deserializer)
        {
            this.header = Std.HeaderMsg.Deserialize(deserializer);
            deserializer.Read(out this.child_frame_id);
            this.pose = Geometry.PoseWithCovarianceMsg.Deserialize(deserializer);
            this.twist = Geometry.TwistWithCovarianceMsg.Deserialize(deserializer);
            this.size = Geometry.Vector3Msg.Deserialize(deserializer);
            deserializer.Read(out this.label);
            deserializer.Read(out this.keypoints, Geometry.PoseMsg.Deserialize, deserializer.ReadLength());
            deserializer.Read(out this.keypoint_names, deserializer.ReadLength());
            deserializer.Read(out this.score);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.child_frame_id);
            serializer.Write(this.pose);
            serializer.Write(this.twist);
            serializer.Write(this.size);
            serializer.Write(this.label);
            serializer.WriteLength(this.keypoints);
            serializer.Write(this.keypoints);
            serializer.WriteLength(this.keypoint_names);
            serializer.Write(this.keypoint_names);
            serializer.Write(this.score);
        }

        public override string ToString()
        {
            return "EstimatedObjectMsg: " +
            "\nheader: " + header.ToString() +
            "\nchild_frame_id: " + child_frame_id.ToString() +
            "\npose: " + pose.ToString() +
            "\ntwist: " + twist.ToString() +
            "\nsize: " + size.ToString() +
            "\nlabel: " + label.ToString() +
            "\nkeypoints: " + System.String.Join(", ", keypoints.ToList()) +
            "\nkeypoint_names: " + System.String.Join(", ", keypoint_names.ToList()) +
            "\nscore: " + score.ToString();
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
