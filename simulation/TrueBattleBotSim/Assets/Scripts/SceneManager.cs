using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SceneManager : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        ObjectiveConfig objective = ConfigManager.LoadObjective("auto_far_side.json");
        ScenarioConfig scenario = ConfigManager.LoadScenario("test1.json");

        Debug.Log($"init x: {objective.init.x}");
        Debug.Log($"scenario: {scenario.cage.dims.x}, {scenario.cage.dims.y}");
    }

    // Update is called once per frame
    void Update()
    {

    }
}
