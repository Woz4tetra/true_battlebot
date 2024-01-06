using UnityEngine;
using RosMessageTypes.Geometry;
class KeyboardInput : MonoBehaviour
{
    [SerializeField] private float linearScale = 1.0f;
    [SerializeField] private float angularScale = 3.0f;

    private ControllerInterface controller;
    public void Start()
    {
        controller = GetComponent<ControllerInterface>();
    }
    public void Update()
    {
        updateCommand();
    }

    private void updateCommand()
    {

        float angular = 0.0f;
        float linear = 0.0f;
        if (Input.GetKey(KeyCode.A))
        {
            angular += angularScale;
        }

        if (Input.GetKey(KeyCode.D))
        {
            angular += -angularScale;
        }

        if (Input.GetKey(KeyCode.W))
        {
            linear += linearScale;
        }

        if (Input.GetKey(KeyCode.S))
        {
            linear += -linearScale;
        }
        TwistMsg command = new TwistMsg
        {
            linear = new Vector3Msg { x = linear },
            angular = new Vector3Msg { z = angular }
        };
        controller.setCommand(command);
    }
}
