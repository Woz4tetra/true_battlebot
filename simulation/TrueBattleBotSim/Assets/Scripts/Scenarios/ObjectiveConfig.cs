using System;
using System.Collections.Generic;
using UnityEngine;

[Serializable]
public class ObjectiveConfig
{
    public string type = "idle";
    public string follower_engine = "PID";  // PID, Ramsete, Teleport, TeleportSmooth
    public bool smooth_teleports = false;
    public ScenarioInitConfig init = new ScenarioInitConfig();
    public List<SequenceElementConfig> sequence = new List<SequenceElementConfig>();

    public string ToJson()
    {
        return JsonUtility.ToJson(this);
    }

    public static ObjectiveConfig FromJson(string json)
    {
        try
        {
            return JsonUtility.FromJson<ObjectiveConfig>(json);
        }
        catch (ArgumentException e)
        {
            Debug.LogError("Error parsing JSON: " + e.Message);
            return new ObjectiveConfig();
        }
    }

    public static ObjectiveConfig FromJsonFile(string path)
    {
        if (!System.IO.File.Exists(path))
        {
            Debug.LogError("File not found: " + path);
            return new ObjectiveConfig();
        }
        return FromJson(System.IO.File.ReadAllText(path));
    }
}
