using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class ConfigManager
{
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
        string path = Path.Combine(Application.persistentDataPath, "Scenarios", scenarioName + ".json");
        if (!File.Exists(path))
        {
            throw new FileNotFoundException($"Scenario file not found: {path}");
        }
        ScenarioConfig scenario = ScenarioConfig.FromJson(File.ReadAllText(path));
        return scenario;
    }

    public static ObjectiveConfig LoadObjective(string objectiveName)
    {
        string path = Path.Combine(Application.persistentDataPath, "Objectives", objectiveName + ".json");
        if (!File.Exists(path))
        {
            throw new FileNotFoundException($"Objective file not found: {path}");
        }
        ObjectiveConfig objective = ObjectiveConfig.FromJson(File.ReadAllText(path));
        return objective;
    }
}