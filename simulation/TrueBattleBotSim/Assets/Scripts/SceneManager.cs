using System.Collections.Generic;
using System.IO;
using System.Linq;
using MathExtensions;
using UnityEngine;

public class SceneManager : MonoBehaviour
{
    [SerializeField] GameObject scaleableField;
    [SerializeField] string baseDirectory = "Config";
    [SerializeField] string scenariosDirectory = "Scenarios";
    [SerializeField] string objectiveDirectory = "Objectives";

    ScenarioConfig scenario;
    Dictionary<string, GameObject> robot_prefabs = new Dictionary<string, GameObject>();
    Dictionary<string, GameObject> active_robots = new Dictionary<string, GameObject>();
    Dictionary<string, ObjectiveConfig> objectives = new Dictionary<string, ObjectiveConfig>();

    void Start()
    {
        Debug.Log($"Current directory: {Application.dataPath}");
        Debug.Log($"Scenarios: {GetScenarioNames()}");
    }

    public string[] GetScenarioNames()
    {
        TextAsset fileNamesAsset = Resources.Load<TextAsset>("FileNames");
        FileNameInfo fileInfoLoaded = JsonUtility.FromJson<FileNameInfo>(fileNamesAsset.text);
        return fileInfoLoaded.GetFiles(Path.Combine(baseDirectory, scenariosDirectory));
    }

    public void LoadScenario(string scenarioName)
    {
        foreach (GameObject robot in active_robots.Values)
        {
            Destroy(robot);
        }
        active_robots.Clear();
        if (scenarioName.Length == 0)
        {
            return;
        }

        scenario = ConfigManager.LoadScenario(scenarioName);
        GameObject[] robot_list = Resources.LoadAll<GameObject>("Prefabs/Robots");
        if (robot_list.Length == 0)
        {
            Debug.LogError("No robot prefabs found");
        }
        foreach (GameObject robot in robot_list)
        {
            Debug.Log($"Loaded robot prefab: {robot.name}");
            robot_prefabs[robot.name] = robot;
        }

        scaleableField.transform.localScale = new Vector3(scenario.cage.dims.x, 1, scenario.cage.dims.y);

        foreach (RobotConfig robot_config in scenario.robots)
        {
            ObjectiveConfig objective_config = ConfigManager.LoadObjective(robot_config.objective);
            objectives[robot_config.name] = objective_config;
            Bounds robot_bounds = GetMaxBounds(robot_prefabs[robot_config.model]);
            Matrix4x4 robot_pose = GetPoseFromConfig(objective_config.init, scenario.cage.dims, robot_bounds);
            GameObject robot = Instantiate(robot_prefabs[robot_config.model], robot_pose.GetT(), robot_pose.GetR());
            active_robots[robot_config.name] = robot;
        }
    }

    Bounds GetMaxBounds(GameObject obj)
    {
        var bounds = new Bounds(obj.transform.position, Vector3.zero);
        foreach (Renderer render in obj.GetComponentsInChildren<Renderer>())
        {
            bounds.Encapsulate(render.bounds);
        }
        return bounds;
    }

    Matrix4x4 GetPoseFromConfig(ScenarioInitConfig init_config, DimsConfig dims_config, Bounds robot_bounds)
    {
        Vector2 scale;
        switch (init_config.type)
        {
            case "absolute":
                scale = Vector2.one;
                break;
            case "relative":
                scale = new Vector2(dims_config.x / 2, dims_config.y / 2);
                break;
            default:
                scale = Vector2.one;
                Debug.LogError("Invalid pose type: " + init_config.type);
                break;
        }
        float height = Mathf.Max(robot_bounds.center.x, Mathf.Max(robot_bounds.center.y, robot_bounds.center.z));
        return Matrix4x4.TRS(
            new Vector3(init_config.x * scale.x, height, init_config.y * scale.y),
            Quaternion.Euler(0, init_config.theta, 0),
            Vector3.one
        );
    }

    // Update is called once per frame
    void Update()
    {

    }
}