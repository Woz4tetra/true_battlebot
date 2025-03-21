//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.BwInterfaces
{
    [Serializable]
    public class GoalEngineConfigMsg : Message
    {
        public const string k_RosMessageName = "bw_interfaces/GoalEngineConfig";
        public override string RosMessageName => k_RosMessageName;

        public double max_velocity;
        public double max_angular_velocity;
        public double max_acceleration;
        public double max_centripetal_acceleration;
        public bool is_max_centripetal_acceleration;
        public bool rotate_at_end;
        public double start_velocity;
        public bool is_start_velocity;
        public double end_velocity;
        public bool is_end_velocity;

        public GoalEngineConfigMsg()
        {
            this.max_velocity = 0.0;
            this.max_angular_velocity = 0.0;
            this.max_acceleration = 0.0;
            this.max_centripetal_acceleration = 0.0;
            this.is_max_centripetal_acceleration = false;
            this.rotate_at_end = false;
            this.start_velocity = 0.0;
            this.is_start_velocity = false;
            this.end_velocity = 0.0;
            this.is_end_velocity = false;
        }

        public GoalEngineConfigMsg(double max_velocity, double max_angular_velocity, double max_acceleration, double max_centripetal_acceleration, bool is_max_centripetal_acceleration, bool rotate_at_end, double start_velocity, bool is_start_velocity, double end_velocity, bool is_end_velocity)
        {
            this.max_velocity = max_velocity;
            this.max_angular_velocity = max_angular_velocity;
            this.max_acceleration = max_acceleration;
            this.max_centripetal_acceleration = max_centripetal_acceleration;
            this.is_max_centripetal_acceleration = is_max_centripetal_acceleration;
            this.rotate_at_end = rotate_at_end;
            this.start_velocity = start_velocity;
            this.is_start_velocity = is_start_velocity;
            this.end_velocity = end_velocity;
            this.is_end_velocity = is_end_velocity;
        }

        public static GoalEngineConfigMsg Deserialize(MessageDeserializer deserializer) => new GoalEngineConfigMsg(deserializer);

        private GoalEngineConfigMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.max_velocity);
            deserializer.Read(out this.max_angular_velocity);
            deserializer.Read(out this.max_acceleration);
            deserializer.Read(out this.max_centripetal_acceleration);
            deserializer.Read(out this.is_max_centripetal_acceleration);
            deserializer.Read(out this.rotate_at_end);
            deserializer.Read(out this.start_velocity);
            deserializer.Read(out this.is_start_velocity);
            deserializer.Read(out this.end_velocity);
            deserializer.Read(out this.is_end_velocity);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.max_velocity);
            serializer.Write(this.max_angular_velocity);
            serializer.Write(this.max_acceleration);
            serializer.Write(this.max_centripetal_acceleration);
            serializer.Write(this.is_max_centripetal_acceleration);
            serializer.Write(this.rotate_at_end);
            serializer.Write(this.start_velocity);
            serializer.Write(this.is_start_velocity);
            serializer.Write(this.end_velocity);
            serializer.Write(this.is_end_velocity);
        }

        public override string ToString()
        {
            return "GoalEngineConfigMsg: " +
            "\nmax_velocity: " + max_velocity.ToString() +
            "\nmax_angular_velocity: " + max_angular_velocity.ToString() +
            "\nmax_acceleration: " + max_acceleration.ToString() +
            "\nmax_centripetal_acceleration: " + max_centripetal_acceleration.ToString() +
            "\nis_max_centripetal_acceleration: " + is_max_centripetal_acceleration.ToString() +
            "\nrotate_at_end: " + rotate_at_end.ToString() +
            "\nstart_velocity: " + start_velocity.ToString() +
            "\nis_start_velocity: " + is_start_velocity.ToString() +
            "\nend_velocity: " + end_velocity.ToString() +
            "\nis_end_velocity: " + is_end_velocity.ToString();
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
