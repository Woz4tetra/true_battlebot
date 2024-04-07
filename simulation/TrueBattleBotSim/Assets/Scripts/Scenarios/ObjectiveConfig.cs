using System;
using System.Collections.Generic;
using UnityEngine;

[Serializable]
public class ObjectiveConfig
{
    public string type;
    public ScenarioInitConfig init;
    public List<SequenceElementConfig> sequence;

    public string ToJson()
    {
        return JsonUtility.ToJson(this);
    }

    public static ObjectiveConfig FromJson(string json)
    {
        return JsonUtility.FromJson<ObjectiveConfig>(json);
    }

    public static ObjectiveConfig FromJsonFile(string path)
    {
        if (!System.IO.File.Exists(path))
        {
            throw new System.IO.FileNotFoundException("File not found: " + path);
        }
        return FromJson(System.IO.File.ReadAllText(path));
    }
}
