using System.IO;
using System.Text;
using UnityEngine;

public class ConfigManager
{
    public static ScenarioConfig LoadScenario(string scenarioName)
    {
        scenarioName = scenarioName.Replace(".json", "");
        string path = $"Scenarios/{scenarioName}";
        TextAsset asset = Resources.Load<TextAsset>(path);
        ScenarioConfig scenario = ScenarioConfig.FromJson(asset.text);
        return scenario;
    }

    public static ObjectiveConfig LoadObjective(string objectiveName)
    {
        objectiveName = objectiveName.Replace(".json", "");
        string path = $"Objectives/{objectiveName}";
        TextAsset asset = Resources.Load<TextAsset>(path);
        ObjectiveConfig objective = ObjectiveConfig.FromJson(asset.text);
        return objective;
    }
}