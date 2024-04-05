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
}
