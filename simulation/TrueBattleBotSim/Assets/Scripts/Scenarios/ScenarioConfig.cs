using System;
using System.Collections.Generic;
using UnityEngine;

[Serializable]
public class ScenarioConfig
{
    public CageConfig cage = new CageConfig();
    public List<RobotConfig> robots = new List<RobotConfig>();

    public string ToJson()
    {
        return JsonUtility.ToJson(this);
    }

    public static ScenarioConfig FromJson(string json)
    {
        return JsonUtility.FromJson<ScenarioConfig>(json);
    }

    public static ScenarioConfig FromJsonFile(string path)
    {
        if (!System.IO.File.Exists(path))
        {
            throw new System.IO.FileNotFoundException("File not found: " + path);
        }
        return FromJson(System.IO.File.ReadAllText(path));
    }
}