using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using MathExtensions;
using UnityEngine;

public class FieldManager : MonoBehaviour
{
    [SerializeField] GameObject scaleableField;
    [SerializeField] GameObject slowCam;
    [SerializeField] GameObject trackingCam;
    [SerializeField] string baseDirectory = "Config";
    [SerializeField] string scenariosDirectory = "Scenarios";
    [SerializeField] GameObject flatScreenTV;
    [SerializeField] float maxCageSize = 5.0f;
    [SerializeField] float heightFudgeFactor = 1.1f;
    PauseManager pauseManager;
    ScenarioConfig scenario;
    string currentScenarioName = "";
    GameObject[] robot_list;
    Dictionary<string, GameObject> robot_prefabs = new Dictionary<string, GameObject>();
    Dictionary<string, GameObject> active_robots = new Dictionary<string, GameObject>();
    Dictionary<string, ObjectiveConfig> objectives = new Dictionary<string, ObjectiveConfig>();
    bool keyboard_been_set = false;

    void Start()
    {
        pauseManager = transform.Find("PauseManager").GetComponent<PauseManager>();
        if (pauseManager == null)
        {
            Debug.LogError("PauseManager not found");
        }
        Debug.Log($"Current directory: {Application.dataPath}");
        Debug.Log($"Scenarios: {GetScenarioNames()}");
        robot_list = Resources.LoadAll<GameObject>("Robots");
        if (robot_list.Length == 0)
        {
            Debug.LogError("No robot prefabs found");
        }
        foreach (GameObject robot in robot_list)
        {
            Debug.Log($"Loaded robot prefab: {robot.name}");
            robot_prefabs[robot.name] = robot;
        }
    }

    public string[] GetScenarioNames()
    {
        TextAsset fileNamesAsset = Resources.Load<TextAsset>("FileNames");
        FileNameInfo fileInfoLoaded = JsonUtility.FromJson<FileNameInfo>(fileNamesAsset.text);
        return fileInfoLoaded.GetFiles(Path.Combine(baseDirectory, scenariosDirectory));
    }

    public void LoadScenario(string scenarioName)
    {
        Debug.Log($"Loading scenario: {scenarioName}");
        currentScenarioName = scenarioName;
        foreach (GameObject robot in active_robots.Values)
        {
            robot.SetActive(false);
        }
        if (scenarioName.Length == 0)
        {
            Debug.Log("No scenario selected");
            return;
        }

        scenario = ConfigManager.LoadScenario(scenarioName);

        scenario.cage.dims.x = Mathf.Min(scenario.cage.dims.x, maxCageSize);
        scenario.cage.dims.y = Mathf.Min(scenario.cage.dims.y, maxCageSize);

        scaleableField.transform.localScale = new Vector3(scenario.cage.dims.x, 1, scenario.cage.dims.y);
        SetObjectPose(slowCam, scenario.cage.slow_cam);
        SetObjectPose(trackingCam, scenario.cage.tracking_cam);
        Bounds field_bounds = GetMaxBounds(scaleableField);
        Debug.Log($"Field bounds: {field_bounds.size}");
        flatScreenTV.transform.position = new Vector3(
            flatScreenTV.transform.position.x,
            flatScreenTV.transform.position.y,
            -scenario.cage.dims.y / 2 * 1.12f
        );

        keyboard_been_set = false;
        foreach (RobotConfig robot_config in scenario.robots)
        {
            if (robot_config.objective.Length == 0)
            {
                robot_config.objective = "idle";
            }
            ObjectiveConfig objective_config = ConfigManager.LoadObjective(robot_config.objective);
            objectives[robot_config.name] = objective_config;
            Bounds robot_bounds = GetMaxBounds(robot_prefabs[robot_config.model]);
            Matrix4x4 robot_pose = GetPoseFromConfig(objective_config.init, scenario.cage.dims, robot_bounds);
            GameObject robot;
            if (active_robots.ContainsKey(robot_config.name))
            {
                robot = active_robots[robot_config.name];
                robot.transform.position = robot_pose.GetT();
                robot.transform.rotation = robot_pose.GetR();
            }
            else
            {
                robot = Instantiate(robot_prefabs[robot_config.model], robot_pose.GetT(), robot_pose.GetR());
                active_robots[robot_config.name] = robot;
            }
            ActivateRobotType(robot, objective_config);
            robot.SetActive(true);
        }
    }

    public void ReloadScenario()
    {
        Debug.Log($"Reloading scenario: {currentScenarioName}");
        LoadScenario(currentScenarioName);
    }

    public PauseManager GetPauseManager()
    {
        return pauseManager;
    }

    void ActivateRobotType(GameObject robot, ObjectiveConfig objective_config)
    {
        KeyboardInput keyboard_input = robot.GetComponent<KeyboardInput>();
        RosControllerConnector controller = robot.GetComponent<RosControllerConnector>();
        WaypointFollower waypoint_follower = robot.GetComponent<WaypointFollower>();
        TargetFollower target_follower = robot.GetComponent<TargetFollower>();
        try
        {
            keyboard_input.enabled = false;
        }
        catch (NullReferenceException e)
        {
            Debug.LogError($"Robot {robot.name} prefab missing keyboard input: {e.Message}");
        }
        try
        {
            controller.enabled = false;
        }
        catch (NullReferenceException e)
        {
            Debug.LogError($"Robot {robot.name} prefab missing controller input: {e.Message}");
        }
        try
        {
            waypoint_follower.enabled = false;
        }
        catch (NullReferenceException e)
        {
            Debug.LogError($"Robot {robot.name} prefab missing follower input: {e.Message}");
        }
        try
        {
            target_follower.enabled = false;
        }
        catch (NullReferenceException e)
        {
            Debug.LogError($"Robot {robot.name} prefab missing target input: {e.Message}");
        }

        switch (objective_config.type)
        {
            case "keyboard":
                if (keyboard_been_set)
                {
                    Debug.LogWarning("Multiple keyboard objectives detected");
                }
                keyboard_input.enabled = true;
                keyboard_been_set = true;
                break;
            case "idle":
                break;
            case "auto":
                controller.enabled = true;
                break;
            case "follow":
                waypoint_follower.enabled = true;
                waypoint_follower.SetSequence(GetScaledSequence(objective_config.init, objective_config.sequence));
                break;
            case "target":
                target_follower.enabled = true;
                target_follower.SetSequence(objective_config.sequence);
                target_follower.SetActiveRobots(active_robots);
                break;
            default:
                Debug.LogError("Invalid objective type: " + objective_config.type);
                break;
        }
    }

    List<SequenceElementConfig> GetScaledSequence(ScenarioInitConfig init_config, List<SequenceElementConfig> sequence)
    {
        List<SequenceElementConfig> scaled_sequence = new List<SequenceElementConfig>();
        float x_scale = scenario.cage.dims.x / 2 * (1.0f - init_config.x_buffer);
        float y_scale = scenario.cage.dims.y / 2 * (1.0f - init_config.y_buffer);
        foreach (SequenceElementConfig element in sequence)
        {
            scaled_sequence.Add(new SequenceElementConfig
            {
                timestamp = element.timestamp,
                x = element.x * x_scale,
                y = element.y * y_scale,
                theta = -element.theta + 90.0f,
                vx = element.vx * x_scale,
                vy = element.vy * y_scale,
                vtheta = element.vtheta
            });
        }
        return scaled_sequence;
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
        float height = Mathf.Min(robot_bounds.extents.x, Mathf.Min(robot_bounds.extents.y, robot_bounds.extents.z));
        height *= heightFudgeFactor;
        return Matrix4x4.TRS(
            new Vector3(
                init_config.x * scale.x * (1.0f - init_config.x_buffer),
                height,
                init_config.y * scale.y * (1.0f - init_config.y_buffer)),
            Quaternion.Euler(0, -1 * init_config.theta, 0),
            Vector3.one
        );
    }

    void SetObjectPose(GameObject obj, PoseConfig pose)
    {
        obj.transform.position = new Vector3(pose.position.x, pose.position.y, pose.position.z);
        obj.transform.rotation = Quaternion.Euler(pose.rotation.x, pose.rotation.y, pose.rotation.z);
    }

    // Update is called once per frame
    void Update()
    {

    }
}
