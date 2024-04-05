using System;
using System.Collections.Generic;
using UnityEngine;

[Serializable]
public class ScenarioConfig
{
    public CageConfig cage;
    public List<RobotConfig> robots;

    public string ToJson()
    {
        return JsonUtility.ToJson(this);
    }

    public static ScenarioConfig FromJson(string json)
    {
        return JsonUtility.FromJson<ScenarioConfig>(json);
    }
}