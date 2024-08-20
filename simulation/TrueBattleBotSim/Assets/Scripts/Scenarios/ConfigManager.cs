using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class ConfigManager
{
    public static Dictionary<string, ScenarioConfig> scenarios = new Dictionary<string, ScenarioConfig>();
    public static Dictionary<string, ObjectiveConfig> objectives = new Dictionary<string, ObjectiveConfig>();

    public static string[] GetScenarioNames()
    {
        if (Directory.Exists(Application.persistentDataPath))
        {
            string scenariosFolder = Path.Combine(Application.persistentDataPath, "Scenarios");

            DirectoryInfo directory = new DirectoryInfo(scenariosFolder);
            List<string> scenarioPaths = new List<string>();
            foreach (var file in directory.GetFiles("*.json"))
            {
                scenarioPaths.Add(Path.GetFileNameWithoutExtension(file.Name));
            }
            return scenarioPaths.ToArray();
        }
        else
        {
            File.Create(Application.persistentDataPath);
            return new string[0];
        }
    }

    public static ScenarioConfig LoadScenario(string scenarioName)
    {
        if (scenarios.ContainsKey(scenarioName))
        {
            return scenarios[scenarioName];
        }
        string path = Path.Combine(Application.persistentDataPath, "Scenarios", scenarioName + ".json");
        if (!File.Exists(path))
        {
            throw new FileNotFoundException($"Scenario file not found: {path}");
        }
        ScenarioConfig scenario = ScenarioConfig.FromJson(File.ReadAllText(path));
        scenarios[scenarioName] = scenario;
        return scenario;
    }

    public static ObjectiveConfig LoadObjective(string objectiveName)
    {
        if (objectives.ContainsKey(objectiveName))
        {
            return objectives[objectiveName];
        }
        string path = Path.Combine(Application.persistentDataPath, "Objectives", objectiveName + ".json");
        if (!File.Exists(path))
        {
            throw new FileNotFoundException($"Objective file not found: {path}");
        }
        ObjectiveConfig objective = ObjectiveConfig.FromJson(File.ReadAllText(path));
        objectives[objectiveName] = objective;
        return objective;
    }

    public static void AddScenarioFromJson(string scenarioName, string scenarioJson)
    {
        ScenarioConfig scenario = ScenarioConfig.FromJson(scenarioJson);
        AddScenario(scenarioName, scenario);
    }

    public static void AddScenario(string scenarioName, ScenarioConfig scenario)
    {
        scenarios[scenarioName] = scenario;
    }

    public static void AddObjectiveFromJson(string objectiveName, string objectiveJson)
    {
        ObjectiveConfig objective = ObjectiveConfig.FromJson(objectiveJson);
        AddObjective(objectiveName, objective);
    }

    public static void AddObjective(string objectiveName, ObjectiveConfig objective)
    {
        objectives[objectiveName] = objective;
    }
}