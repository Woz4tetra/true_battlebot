using UnityEngine;

class Wheel : MonoBehaviour
{
    [SerializeField] private float maxWheelSpeed = 1.0f;
    [SerializeField] private float wheelRadius = 1.0f;
    private ArticulationBody body;
    private float angularVelocity = 0.0f;

    void Start()
    {
        body = GetComponent<ArticulationBody>();
    }

    public void setVelocity(float groundVelocity)
    {
        if (Mathf.Abs(groundVelocity) > maxWheelSpeed)
        {
            groundVelocity = Mathf.Sign(groundVelocity) * maxWheelSpeed;
        }
        angularVelocity = Mathf.Rad2Deg * groundVelocity / wheelRadius;
    }

    void FixedUpdate()
    {
        ArticulationDrive drive = body.xDrive;
        drive.targetVelocity = angularVelocity;
        body.xDrive = drive;
    }
}