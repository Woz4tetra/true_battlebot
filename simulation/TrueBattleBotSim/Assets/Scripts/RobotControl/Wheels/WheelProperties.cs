using UnityEngine;

public class WheelProperties : MonoBehaviour
{
    [SerializeField] private float maxWheelSpeedRpm = 1000.0f;
    [SerializeField] private float wheelRadius = 1.0f;
    [SerializeField] private float accelerationRpmPerSec = 100.0f;
    [SerializeField] private float decelerationRpmPerSec = 10000.0f;

    public float MaxWheelSpeedRpm { get { return maxWheelSpeedRpm; } }
    public float WheelRadius { get { return wheelRadius; } }
    public float AccelerationRpmPerSec { get { return accelerationRpmPerSec; } }
    public float DecelerationRpmPerSec { get { return decelerationRpmPerSec; } }
}
