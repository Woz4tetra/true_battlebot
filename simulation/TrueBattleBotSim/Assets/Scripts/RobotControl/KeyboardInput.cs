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
        TwistMsg twist = controller.getGroundTruth().twist.twist;
        Debug.Log($"odom: {twist.linear.x}, {twist.angular.z}");
    }

    private void updateCommand()
    {
        float linear = Input.GetAxis("Vertical") * linearScale;
        float angular = Input.GetAxis("Horizontal") * -angularScale;
        TwistMsg command = new TwistMsg
        {
            linear = new Vector3Msg { x = linear },
            angular = new Vector3Msg { z = angular }
        };
        Debug.Log($"cmd: {command.linear.x}, {command.angular.z}");
        controller.setCommand(command);
    }
}
