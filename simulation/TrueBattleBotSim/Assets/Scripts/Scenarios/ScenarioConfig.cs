using System;
using System.Collections.Generic;
using UnityEngine;

[Serializable]
public class ScenarioConfig
{
    public CageConfig cage = new CageConfig();
    public BackgroundConfig background = new BackgroundConfig();

    public CameraConfig main_cam = new CameraConfig
    {
        init = new ScenarioInitConfig
        {
            type = "world",
            x = 0.0f,
            y = 1.167f,
            z = -2.05f,
            roll = 29.654f,
            pitch = 0.0f,
            yaw = 0.0f
        }
    };
    public List<ActorConfig> actors = new List<ActorConfig>();
    public FixturesConfig fixtures = new FixturesConfig();

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