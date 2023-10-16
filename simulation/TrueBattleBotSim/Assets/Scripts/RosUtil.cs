using System;
using RosMessageTypes.BuiltinInterfaces;

class RosUtil
{
    public static TimeMsg GetTimeMsg() {
        DateTime epochStart = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);
        double currentEpochTime = (DateTime.UtcNow - epochStart).TotalSeconds;
        uint currentSeconds = (uint)currentEpochTime;
        uint currentNanoSeconds = (uint)((currentEpochTime % 1.0) * 1E9);
        return new TimeMsg {
            sec = currentSeconds,
            nanosec = currentNanoSeconds
        };
    }
}