//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.BwInterfaces
{
    [Serializable]
    public class ConfigureSimulationMsg : Message
    {
        public const string k_RosMessageName = "bw_interfaces/ConfigureSimulation";
        public override string RosMessageName => k_RosMessageName;

        public SimulationConfigMsg scenario;
        public SimulationConfigMsg[] objectives;

        public ConfigureSimulationMsg()
        {
            this.scenario = new SimulationConfigMsg();
            this.objectives = new SimulationConfigMsg[0];
        }

        public ConfigureSimulationMsg(SimulationConfigMsg scenario, SimulationConfigMsg[] objectives)
        {
            this.scenario = scenario;
            this.objectives = objectives;
        }

        public static ConfigureSimulationMsg Deserialize(MessageDeserializer deserializer) => new ConfigureSimulationMsg(deserializer);

        private ConfigureSimulationMsg(MessageDeserializer deserializer)
        {
            this.scenario = SimulationConfigMsg.Deserialize(deserializer);
            deserializer.Read(out this.objectives, SimulationConfigMsg.Deserialize, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.scenario);
            serializer.WriteLength(this.objectives);
            serializer.Write(this.objectives);
        }

        public override string ToString()
        {
            return "ConfigureSimulationMsg: " +
            "\nscenario: " + scenario.ToString() +
            "\nobjectives: " + System.String.Join(", ", objectives.ToList());
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