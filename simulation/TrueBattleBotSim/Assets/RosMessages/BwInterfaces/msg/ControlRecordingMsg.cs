//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.BwInterfaces
{
    [Serializable]
    public class ControlRecordingMsg : Message
    {
        public const string k_RosMessageName = "bw_interfaces/ControlRecording";
        public override string RosMessageName => k_RosMessageName;

        public const int START = 0;
        public const int STOP = 1;
        public Std.HeaderMsg header;
        //  The name of the recording
        public string name;
        //  Command to start or stop recording
        public int command;

        public ControlRecordingMsg()
        {
            this.header = new Std.HeaderMsg();
            this.name = "";
            this.command = 0;
        }

        public ControlRecordingMsg(Std.HeaderMsg header, string name, int command)
        {
            this.header = header;
            this.name = name;
            this.command = command;
        }

        public static ControlRecordingMsg Deserialize(MessageDeserializer deserializer) => new ControlRecordingMsg(deserializer);

        private ControlRecordingMsg(MessageDeserializer deserializer)
        {
            this.header = Std.HeaderMsg.Deserialize(deserializer);
            deserializer.Read(out this.name);
            deserializer.Read(out this.command);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.name);
            serializer.Write(this.command);
        }

        public override string ToString()
        {
            return "ControlRecordingMsg: " +
            "\nheader: " + header.ToString() +
            "\nname: " + name.ToString() +
            "\ncommand: " + command.ToString();
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
