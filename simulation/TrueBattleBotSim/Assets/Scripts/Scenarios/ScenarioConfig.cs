using System;
using System.Collections.Generic;
using UnityEngine;

[Serializable]
public class ScenarioConfig
{
    public CageConfig cage = new CageConfig();

    public CameraConfig main_cam = new CameraConfig
    {
        pose = new PoseConfig
        {
            position = new PositionConfig
            {
                x = 0.0f,
                y = 1.167f,
                z = -2.05f
            },
            rotation = new RotationConfig
            {
                x = 29.654f,
                y = 0.0f,
                z = 0.0f
            }
        }
    };
    public List<ActorConfig> actors = new List<ActorConfig>();

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