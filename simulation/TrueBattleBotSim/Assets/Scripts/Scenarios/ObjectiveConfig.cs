using System;
using System.Collections.Generic;
using UnityEngine;

[Serializable]
public class ObjectiveConfig
{
    public string type = "idle";  // keyboard, idle, auto, follow, target, teleport, relative_to
    public string follower_engine = "PID";  // PID, Ramsete, Teleport
    public bool smooth_teleports = false;
    public ScenarioInitConfig init = new ScenarioInitConfig();
    public List<SequenceElementConfig> sequence = new List<SequenceElementConfig>();

    public bool overwrite_controller_config = false;
    public PidConfig linear_pid = new PidConfig(2.0f, 0.0f, 0.0f, 1.0f);
    public PidConfig angular_pid = new PidConfig(10.0f, 0.1f, 1.0f, 1.0f);
    public RamseteConfig ramsete = new RamseteConfig(2.0f, 0.7f);

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
