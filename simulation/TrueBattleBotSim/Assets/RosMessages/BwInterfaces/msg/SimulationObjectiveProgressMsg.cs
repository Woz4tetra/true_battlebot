//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.BuiltinInterfaces;

namespace RosMessageTypes.BwInterfaces
{
    [Serializable]
    public class SimulationObjectiveProgressMsg : Message
    {
        public const string k_RosMessageName = "bw_interfaces/SimulationObjectiveProgress";
        public override string RosMessageName => k_RosMessageName;

        public Std.HeaderMsg header;
        public TimeMsg duration;
        public uint objective_index;
        public uint sequence_length;
        public string objective_name;

        public SimulationObjectiveProgressMsg()
        {
            this.header = new Std.HeaderMsg();
            this.duration = new TimeMsg();
            this.objective_index = 0;
            this.sequence_length = 0;
            this.objective_name = "";
        }

        public SimulationObjectiveProgressMsg(Std.HeaderMsg header, TimeMsg duration, uint objective_index, uint sequence_length, string objective_name)
        {
            this.header = header;
            this.duration = duration;
            this.objective_index = objective_index;
            this.sequence_length = sequence_length;
            this.objective_name = objective_name;
        }

        public static SimulationObjectiveProgressMsg Deserialize(MessageDeserializer deserializer) => new SimulationObjectiveProgressMsg(deserializer);

        private SimulationObjectiveProgressMsg(MessageDeserializer deserializer)
        {
            this.header = Std.HeaderMsg.Deserialize(deserializer);
            this.duration = TimeMsg.Deserialize(deserializer);
            deserializer.Read(out this.objective_index);
            deserializer.Read(out this.sequence_length);
            deserializer.Read(out this.objective_name);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.duration);
            serializer.Write(this.objective_index);
            serializer.Write(this.sequence_length);
            serializer.Write(this.objective_name);
        }

        public override string ToString()
        {
            return "SimulationObjectiveProgressMsg: " +
            "\nheader: " + header.ToString() +
            "\nduration: " + duration.ToString() +
            "\nobjective_index: " + objective_index.ToString() +
            "\nsequence_length: " + sequence_length.ToString() +
            "\nobjective_name: " + objective_name.ToString();
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