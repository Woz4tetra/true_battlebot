using System.IO;
using UnityEngine;

public class ConfigManager
{
    public static ScenarioConfig LoadScenario(string scenarioName)
    {
        string path = $"Config/Scenarios/{scenarioName}";
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
        string path = $"Config/Objectives/{objectiveName}";
        TextAsset asset = Resources.Load<TextAsset>(path);
        if (asset == null)
        {
            throw new FileNotFoundException($"Objective file not found: {path}");
        }
        ObjectiveConfig objective = ObjectiveConfig.FromJson(asset.text);
        return objective;
    }
}