using UnityEngine;
using RosMessageTypes.Geometry;

class KeyboardInput : MonoBehaviour
{
    [SerializeField] private float linearScale = 1.0f;
    [SerializeField] private float angularScale = 3.0f;
    [SerializeField] private bool useJoystick = false;

    private ControllerInterface controller;
    public void Start()
    {
        controller = GetComponent<ControllerInterface>();
    }
    public void FixedUpdate()
    {
        updateCommand();
    }
    void OnEnable()
    {
        if (controller == null)
        {
            return;
        }
        controller.Reset();
    }

    private void updateCommand()
    {

        float angular = 0.0f;
        float linearX = 0.0f;
        float linearY = 0.0f;
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
            linearX += linearScale;
        }

        if (Input.GetKey(KeyCode.S))
        {
            linearX += -linearScale;
        }

        if (Input.GetKey(KeyCode.Q))
        {
            linearY += linearScale;
        }
        if (Input.GetKey(KeyCode.E))
        {
            linearY += -linearScale;
        }

        if (useJoystick && angular == 0.0f && linearX == 0.0f)
        {
            linearX = Input.GetAxis("Vertical") * -linearScale;
            angular = Input.GetAxis("Horizontal") * -angularScale;
        }

        TwistMsg command = new TwistMsg
        {
            linear = new Vector3Msg { x = linearX, y = linearY },
            angular = new Vector3Msg { z = angular }
        };
        controller.SetCommand(command);
    }
}
