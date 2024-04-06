using System.IO;
using UnityEngine;

public class ConfigManager
{
    public static ScenarioConfig LoadScenario(string scenarioName)
    {
        string path = $"Scenarios/{scenarioName}";
        path = path.Replace(".json", "");
        TextAsset asset = Resources.Load<TextAsset>(path);
        if (asset == null)
        {
            throw new FileNotFoundException($"Scenario file not found: {path}");
        }
        ScenarioConfig scenario = ScenarioConfig.FromJson(asset.text);
        return scenario;
    }

    public static ObjectiveConfig LoadObjective(string objectiveName)
    {
        string path = $"Objectives/{objectiveName}";
        path = path.Replace(".json", "");
        TextAsset asset = Resources.Load<TextAsset>(path);
        if (asset == null)
        {
            throw new FileNotFoundException($"Objective file not found: {path}");
        }
        ObjectiveConfig objective = ObjectiveConfig.FromJson(asset.text);
        return objective;
    }
}