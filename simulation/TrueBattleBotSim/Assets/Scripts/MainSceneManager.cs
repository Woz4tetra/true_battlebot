using System;
using System.Collections.Generic;
using MathExtensions;
using RosMessageTypes.BwInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using UnityEngine.SceneManagement;

public class MainSceneManager : MonoBehaviour
{
    [SerializeField] GameObject mainCam;
    [SerializeField] GameObject flatScreenTV;
    [SerializeField] float maxCageSize = 5.0f;
    [SerializeField] private GameObject referenceObject;
    [SerializeField] string scenarioLoadedTopic = "simulation/scenario_loaded";

    PauseManager pauseManager;
    ScenarioConfig scenario;
    string currentScenarioName = "";
    GameObject[] actor_list;
    GameObject[] cage_list;
    Dictionary<string, GameObject> actorPrefabs = new Dictionary<string, GameObject>();
    Dictionary<string, GameObject> cagePrefabs = new Dictionary<string, GameObject>();
    Dictionary<string, GameObject> activeActors = new Dictionary<string, GameObject>();
    Dictionary<string, GameObject> persistentActors = new Dictionary<string, GameObject>();
    Dictionary<string, ObjectiveConfig> objectives = new Dictionary<string, ObjectiveConfig>();
    GameObject activeCage;
    bool keyboard_been_set = false;
    BackgroundConfig loadedBackgroundConfig = new BackgroundConfig { name = "" };
    ROSConnection ros;
    FixturesManager fixturesManager;
    PhysicsMaterialConfigurator physicsMaterialConfigurator;

    void Start()
    {
        if (referenceObject == null)
        {
            referenceObject = GameObject.Find("Coordinate Frame");
        }
        flatScreenTV.SetActive(false);
        pauseManager = transform.Find("PauseManager").GetComponent<PauseManager>();
        if (pauseManager == null)
        {
            Debug.LogError("PauseManager not found");
        }

        fixturesManager = transform.Find("FixturesManager").GetComponent<FixturesManager>();
        if (fixturesManager == null)
        {
            Debug.LogError("FixturesManager not found");
        }

        Debug.Log($"Current directory: {Application.dataPath}");
        actor_list = Resources.LoadAll<GameObject>("Actors");
        if (actor_list.Length == 0)
        {
            Debug.LogError("No actor prefabs found");
        }
        foreach (GameObject actor in actor_list)
        {
            Debug.Log($"Loaded actor prefab: {actor.name}");
            actorPrefabs[actor.name] = actor;
        }

        cage_list = Resources.LoadAll<GameObject>("Cages");
        if (cage_list.Length == 0)
        {
            Debug.LogError("No cage prefabs found");
        }
        foreach (GameObject cage in cage_list)
        {
            Debug.Log($"Loaded cage prefab: {cage.name}");
            cagePrefabs[cage.name] = cage;
        }
        persistentActors["main_cam"] = mainCam;

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<SimulationScenarioLoadedEventMsg>(scenarioLoadedTopic);

        physicsMaterialConfigurator = GetComponent<PhysicsMaterialConfigurator>();
    }

    public void LoadBackground(BackgroundConfig backgroundConfig)
    {
        if (backgroundConfig.Equals(loadedBackgroundConfig))
        {
            return;
        }
        string newBackgroundName = backgroundConfig.name;
        if (newBackgroundName.Length == 0)
        {
            Debug.Log("No background selected");
            return;
        }
        UnloadBackground(loadedBackgroundConfig);
        Debug.Log($"Loading background: {newBackgroundName}");
        SceneManager.LoadSceneAsync(newBackgroundName, LoadSceneMode.Additive);
        loadedBackgroundConfig = backgroundConfig;
    }

    private void UnloadBackground(BackgroundConfig backgroundConfig)
    {
        if (backgroundConfig.name.Length == 0)
        {
            Debug.Log("No background selected");
            return;
        }
        Debug.Log($"Unloading background: {backgroundConfig.name}");
        SceneManager.UnloadSceneAsync(backgroundConfig.name);
        loadedBackgroundConfig = new BackgroundConfig { name = "" };
    }

    public BackgroundConfig GetLoadedBackgroundConfig()
    {
        return loadedBackgroundConfig;
    }

    public void LoadScenarioByName(string scenarioName)
    {
        Debug.Log($"Loading scenario: {scenarioName}");
        bool didScenarioChange = currentScenarioName != scenarioName;
        currentScenarioName = scenarioName;
        foreach (GameObject actor in activeActors.Values)
        {
            actor.SetActive(false);
        }
        if (scenarioName.Length == 0)
        {
            Debug.Log("No scenario selected");
            flatScreenTV.SetActive(false);
            if (activeCage != null)
            {
                Destroy(activeCage);
            }
            return;
        }

        scenario = ConfigManager.LoadScenario(scenarioName);

        LoadBackground(scenario.background);
        LoadScenario(scenario, didScenarioChange);
    }

    public void LoadScenario(ScenarioConfig scenario, bool resetMainCamera = false)
    {
        scenario.cage.dims.x = Mathf.Min(scenario.cage.dims.x, maxCageSize);
        scenario.cage.dims.y = Mathf.Min(scenario.cage.dims.y, maxCageSize);

        if (activeCage != null)
        {
            Destroy(activeCage);
        }
        Debug.Log($"Loading cage: {scenario.cage.cage_type}");
        if (!cagePrefabs.ContainsKey(scenario.cage.cage_type))
        {
            Debug.LogError("Failed to load cage. Not loading scenario.");
            return;
        }
        activeCage = Instantiate(cagePrefabs[scenario.cage.cage_type]);
        activeCage.transform.localScale = new Vector3(scenario.cage.dims.x, 1, scenario.cage.dims.y);

        if (resetMainCamera)
        {
            Matrix4x4 cam_pose = GetPoseFromConfig(
                scenario.main_cam.init,
                scenario.cage.dims,
                Matrix4x4.identity
            );
            mainCam.transform.position = cam_pose.GetT();
            mainCam.transform.rotation = cam_pose.GetR();

            CameraController mainController = mainCam.GetComponent<CameraController>();
            mainController.ResetTransform();
        }

        Bounds field_bounds = GetMaxBounds(activeCage);
        Debug.Log($"Field bounds: {field_bounds.size}");
        if (scenario.cage.display_readout)
        {
            flatScreenTV.SetActive(true);
        }
        else
        {
            flatScreenTV.SetActive(false);
        }
        flatScreenTV.transform.position = new Vector3(
            flatScreenTV.transform.position.x,
            flatScreenTV.transform.position.y,
            -scenario.cage.dims.y / 2 * 1.12f
        );

        keyboard_been_set = false;
        foreach (GameObject actor in activeActors.Values)
        {
            Destroy(actor);
        }
        activeActors.Clear();

        Debug.Log($"Loading scenario with {scenario.actors.Count} actors");
        foreach (ActorConfig actor_config in scenario.actors)
        {
            if (actor_config.objective.Length == 0)
            {
                actor_config.objective = "idle";
            }
            ObjectiveConfig objective_config = ConfigManager.LoadObjective(actor_config.objective);
            objectives[actor_config.name] = objective_config;
            Debug.Log($"Loading actor: {actor_config.name}");
            GameObject actorPrefab = actorPrefabs[actor_config.model];
            Transform spawnHere = actorPrefab.transform.Find("SpawnHere");
            Matrix4x4 actor_pose = GetPoseFromConfig(
                objective_config.init,
                scenario.cage.dims,
                Matrix4x4.TRS(
                    spawnHere.localPosition,
                    spawnHere.localRotation,
                    spawnHere.localScale
                )
            );
            GameObject actor = Instantiate(actorPrefab, actor_pose.GetT(), actor_pose.GetR());
            activeActors[actor_config.name] = actor;
            actor.gameObject.name = actor_config.name;
            actor.SetActive(true);
        }

        Dictionary<string, GameObject> combinedActors = new Dictionary<string, GameObject>();
        foreach (KeyValuePair<string, GameObject> entry in activeActors)
        {
            combinedActors[entry.Key] = entry.Value;
        }
        foreach (KeyValuePair<string, GameObject> entry in persistentActors)
        {
            combinedActors[entry.Key] = entry.Value;
        }

        List<SimulationObjectiveProgressMsg> objectiveMsgs = new List<SimulationObjectiveProgressMsg>();
        foreach (ActorConfig actor_config in scenario.actors)
        {
            GameObject actor = activeActors[actor_config.name];
            ObjectiveConfig objective_config = objectives[actor_config.name];
            ActivateActorType(actor_config, actor, objective_config, combinedActors);
            objectiveMsgs.Add(
                new SimulationObjectiveProgressMsg
                {
                    header = new HeaderMsg
                    {
                        frame_id = actor_config.name,
                        stamp = RosUtil.GetTimeMsg(0),
                        seq = 0
                    },
                    sequence_length = (uint)objective_config.sequence.Count,
                    objective_name = actor_config.objective,
                }
            );
        }

        fixturesManager.UpdateFromConfig(scenario.fixtures);

        physicsMaterialConfigurator.ResetMaterials();
        for (int i = 0; i < scenario.physics_materials.Count; i++)
        {
            physicsMaterialConfigurator.ConfigureMaterial(scenario.physics_materials[i]);
        }

        ros.Publish(scenarioLoadedTopic, new SimulationScenarioLoadedEventMsg
        {
            objectives = objectiveMsgs.ToArray(),
            scenario_name = currentScenarioName
        });
    }

    public void ReloadScenario()
    {
        Debug.Log($"Reloading scenario: {currentScenarioName}");
        LoadScenarioByName(currentScenarioName);
    }

    public PauseManager GetPauseManager()
    {
        return pauseManager;
    }

    void ActivateActorType(ActorConfig actor_config, GameObject actor, ObjectiveConfig objective_config, Dictionary<string, GameObject> actors)
    {
        KeyboardInput keyboard_input = actor.GetComponent<KeyboardInput>();
        RosControllerConnector controller = actor.GetComponent<RosControllerConnector>();
        WaypointFollower waypoint_follower = actor.GetComponent<WaypointFollower>();
        TargetFollower target_follower = actor.GetComponent<TargetFollower>();
        TeleportFollower teleport_follower = actor.GetComponent<TeleportFollower>();
        RelativeToFollower relative_to_follower = actor.GetComponent<RelativeToFollower>();

        try { keyboard_input.enabled = false; }
        catch (NullReferenceException e) { Debug.Log($"Actor {actor.name} prefab missing keyboard input: {e.Message}"); }

        try { controller.enabled = false; }
        catch (NullReferenceException e) { Debug.Log($"Actor {actor.name} prefab missing controller input: {e.Message}"); }

        try { waypoint_follower.enabled = false; }
        catch (NullReferenceException e) { Debug.Log($"Actor {actor.name} prefab missing follower input: {e.Message}"); }

        try { target_follower.enabled = false; }
        catch (NullReferenceException e) { Debug.Log($"Actor {actor.name} prefab missing target input: {e.Message}"); }

        try { teleport_follower.enabled = false; }
        catch (NullReferenceException e) { Debug.Log($"Actor {actor.name} prefab missing teleport input: {e.Message}"); }

        try { relative_to_follower.enabled = false; }
        catch (NullReferenceException e) { Debug.Log($"Actor {actor.name} prefab missing relative_to input: {e.Message}"); }

        BaseFollowerEngine followerEngine;

        switch (objective_config.type)
        {
            case "keyboard":
                if (keyboard_been_set)
                {
                    Debug.LogWarning("Multiple keyboard objectives detected");
                }
                if (keyboard_input == null)
                {
                    Debug.LogError("Keyboard input not found");
                    break;
                }
                keyboard_input.enabled = true;
                keyboard_been_set = true;
                break;
            case "idle":
                break;
            case "auto":
                if (controller == null)
                {
                    Debug.LogError("Controller not found");
                    break;
                }
                controller.enabled = true;
                break;
            case "follow":
                if (waypoint_follower == null)
                {
                    Debug.LogError("Waypoint follower not found");
                    break;
                }
                followerEngine = GetFollowerEngine(objective_config.follower_engine, actor, objective_config);
                waypoint_follower.enabled = true;
                waypoint_follower.SetSequence(actor_config.name, actor_config.objective, GetScaledSequence(objective_config.init, objective_config.sequence));
                waypoint_follower.SetFollowerEngine(followerEngine);
                break;
            case "target":
                if (target_follower == null)
                {
                    Debug.LogError("Target follower not found");
                    break;
                }
                followerEngine = GetFollowerEngine(objective_config.follower_engine, actor, objective_config);
                target_follower.enabled = true;
                target_follower.SetSequence(actor_config.name, actor_config.objective, objective_config.sequence);
                target_follower.SetActiveActors(actors);
                target_follower.SetFollowerEngine(followerEngine);
                break;
            case "teleport":
                if (teleport_follower == null)
                {
                    Debug.LogError("Teleport follower not found");
                    break;
                }
                teleport_follower.enabled = true;
                teleport_follower.SetSequence(actor_config.name, actor_config.objective, objective_config.sequence);
                teleport_follower.SetComputeMethod(objective_config.smooth_teleports);
                break;
            case "relative_to":
                if (relative_to_follower == null)
                {
                    Debug.LogError("Relative to follower not found");
                    break;
                }
                relative_to_follower.enabled = true;
                relative_to_follower.SetSequence(actor_config.name, actor_config.objective, objective_config.sequence);
                relative_to_follower.SetActiveActors(actors);
                break;
            default:
                Debug.LogError("Invalid objective type: " + objective_config.type);
                break;
        }
    }

    BaseFollowerEngine GetFollowerEngine(string followerType, GameObject actor, ObjectiveConfig objective_config)
    {
        PIDFollowerEngine pid_follower_engine = actor.GetComponent<PIDFollowerEngine>();
        RamseteFollowerEngine ramsete_follower_engine = actor.GetComponent<RamseteFollowerEngine>();

        try { pid_follower_engine.enabled = false; }
        catch (NullReferenceException e) { Debug.Log($"Actor {actor.name} prefab missing PID follower engine: {e.Message}"); }

        try { ramsete_follower_engine.enabled = false; }
        catch (NullReferenceException e) { Debug.Log($"Actor {actor.name} prefab missing Ramsete follower engine: {e.Message}"); }


        BaseFollowerEngine followerEngine = null;
        switch (followerType)
        {
            case "Ramsete":
                if (ramsete_follower_engine == null)
                {
                    Debug.LogError("Ramsete follower engine not found");
                    break;
                }
                ramsete_follower_engine.enabled = true;
                ramsete_follower_engine.SetRamseteConfig(objective_config.ramsete);
                followerEngine = ramsete_follower_engine;
                break;
            case "PID":
                if (pid_follower_engine == null)
                {
                    Debug.LogError("PID follower engine not found");
                    break;
                }
                pid_follower_engine.enabled = true;
                pid_follower_engine.SetLinearPIDConfig(objective_config.linear_pid);
                pid_follower_engine.SetAngularPIDConfig(objective_config.angular_pid);
                followerEngine = pid_follower_engine;
                break;
            default:
                Debug.LogError("Invalid follower engine type: " + followerType);
                break;
        }
        return followerEngine;
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
                yaw = element.yaw,
                vx = element.vx * x_scale,
                vy = element.vy * y_scale,
                vyaw = element.vyaw
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

    Matrix4x4 GetPoseFromConfig(ScenarioInitConfig init_config, DimsConfig dims_config, Matrix4x4 tf_objectorigin_from_spawn)
    {
        Vector2 scale;
        Vector3 position;
        Quaternion rotation;
        Matrix4x4 objectPose = ComputeConfigPoseFromReference(init_config);
        switch (init_config.type)
        {
            case "absolute":
                scale = Vector2.one;
                position = new Vector3(init_config.x, init_config.z, init_config.y);
                rotation = Quaternion.Euler(-1 * init_config.roll, -1 * init_config.yaw, init_config.pitch);
                break;
            case "relative":
                scale = new Vector2(dims_config.x / 2, dims_config.y / 2);
                position = new Vector3(init_config.x, init_config.z, init_config.y);
                rotation = Quaternion.Euler(-1 * init_config.roll, -1 * init_config.yaw, init_config.pitch);
                break;
            case "world":
                scale = Vector2.one;
                position = new Vector3(init_config.x, init_config.y, init_config.z);
                rotation = Quaternion.Euler(init_config.roll, init_config.pitch, init_config.yaw);
                break;
            case "flu":
                scale = Vector2.one;
                position = objectPose.GetT();
                rotation = objectPose.GetR();
                break;
            default:
                scale = Vector2.one;
                position = objectPose.GetT();
                rotation = objectPose.GetR();
                Debug.LogError("Invalid pose type: " + init_config.type);
                break;
        }
        Matrix4x4 tforigin_from_spawn = Matrix4x4.TRS(
            new Vector3(
                position.x * scale.x * (1.0f - init_config.x_buffer),
                position.y * (1.0f - init_config.z_buffer),
                position.z * scale.y * (1.0f - init_config.y_buffer)),
            rotation,
            Vector3.one
        );
        Matrix4x4 tforigin_from_objectorigin = tforigin_from_spawn * tf_objectorigin_from_spawn.inverse;
        return tforigin_from_objectorigin;
    }

    Matrix4x4 ComputeConfigPoseFromReference(ScenarioInitConfig init_config)
    {
        Quaternion tempRotation = new Vector3(init_config.roll, init_config.pitch, init_config.yaw).FromFLUEulerAngles();
        PoseMsg rosPose = new PoseMsg
        {
            position = new PointMsg(init_config.x, init_config.y, init_config.z),
            orientation = new QuaternionMsg(tempRotation.x, tempRotation.y, tempRotation.z, tempRotation.w)
        };
        Matrix4x4 objectPose = Matrix4x4.TRS(rosPose.position.From<FLU>(), rosPose.orientation.From<FLU>(), Vector3.one);
        if (referenceObject != null)
        {
            objectPose = referenceObject.transform.worldToLocalMatrix * objectPose;
        }
        return Matrix4x4.TRS(Vector3.zero, Quaternion.Euler(0, 90, 0), Vector3.one) * objectPose;
    }

    // Update is called once per frame
    void Update()
    {

    }
}
