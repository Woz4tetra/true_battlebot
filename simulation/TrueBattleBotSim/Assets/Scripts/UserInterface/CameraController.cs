using MathExtensions;
using UnityEngine;

public class CameraController : MonoBehaviour
{
    [SerializeField] GameObject focusObject = null;
    [SerializeField] float smoothSpeed = 20.0f;
    [SerializeField] float azimuthScale = 5.0f;
    [SerializeField] float elevationScale = 2.0f;
    [SerializeField] float distanceSpeed = 30.0f;
    [SerializeField] SerializableTuple<float, float> elevationLimits = new SerializableTuple<float, float>(-Mathf.PI / 4, Mathf.PI / 4);
    [SerializeField] SerializableTuple<float, float> distanceLimits = new SerializableTuple<float, float>(1.0f, 5.0f);
    float azimuthAngle = 0.0f, elevationAngle = 0.0f, distance = 10.0f;
    float clickedAzimuthAngle = 0.0f, clickedElevationAngle = 0.0f;
    private Vector2 prevClickPosition = Vector2.zero;

    private bool prevButtonState = false;
    private bool enableControls = true;

    void initializePolarCoordinates()
    {
        Vector3 direction = transform.position - focusObject.transform.position;
        distance = direction.magnitude;
        azimuthAngle = Mathf.Atan2(direction.z, direction.x);
        elevationAngle = Mathf.Asin(direction.y / distance);
    }

    Matrix4x4 getDesiredTransform()
    {
        float x = distance * Mathf.Cos(elevationAngle) * Mathf.Cos(azimuthAngle);
        float y = distance * Mathf.Sin(elevationAngle);
        float z = distance * Mathf.Cos(elevationAngle) * Mathf.Sin(azimuthAngle);
        Vector3 position = new Vector3(x, y, z);
        Quaternion rotation = Quaternion.LookRotation(focusObject.transform.position - position);
        return Matrix4x4.TRS(position + focusObject.transform.position, rotation, Vector3.one);
    }

    void Start()
    {
        if (focusObject == null)
        {
            focusObject = new GameObject();
            focusObject.transform.position = Vector3.zero;
        }
        initializePolarCoordinates();
    }

    private bool GetMouseDown()
    {
        return Input.GetButton("Fire2");
    }

    public void EnableControls(bool enable)
    {
        enableControls = enable;
    }

    private Vector2 GetMovementVector()
    {
        bool buttonState = GetMouseDown();
        Vector2 mousePosition = Input.mousePosition;
        if (buttonState != prevButtonState)
        {
            prevButtonState = buttonState;
            if (buttonState)
            {
                prevClickPosition = mousePosition;
                clickedAzimuthAngle = azimuthAngle;
                clickedElevationAngle = elevationAngle;
            }
        }

        if (buttonState)
        {
            Vector2 rawDirection = mousePosition - prevClickPosition;
            Vector2 scaledDirection = new Vector2(rawDirection.x / Screen.width, rawDirection.y / Screen.height);
            return Vector2.ClampMagnitude(scaledDirection, 1.0f);
        }
        else
        {
            return Vector2.zero;
        }
    }

    public void ResetTransform()
    {
        initializePolarCoordinates();
        Matrix4x4 desiredTransform = getDesiredTransform();
        transform.SetPositionAndRotation(desiredTransform.GetT(), desiredTransform.GetR());
    }

    void Update()
    {
        if (!enableControls)
        {
            return;
        }
        Vector2 movement = GetMovementVector();
        if (movement.magnitude > 0.01f)
        {
            azimuthAngle = -1 * movement.x * azimuthScale + clickedAzimuthAngle;
            elevationAngle = -1 * movement.y * elevationScale + clickedElevationAngle;
        }
        distance += -1 * Input.GetAxis("Mouse ScrollWheel") * distanceSpeed * Time.fixedDeltaTime;
        distance = Mathf.Clamp(distance, distanceLimits.Item1, distanceLimits.Item2);
        elevationAngle = Mathf.Clamp(elevationAngle, elevationLimits.Item1, elevationLimits.Item2);

        Matrix4x4 desiredTransform = getDesiredTransform();
        Vector3 desiredPosition = desiredTransform.GetT();
        Vector3 smoothedPosition = Vector3.Slerp(transform.position, desiredPosition, smoothSpeed * Time.fixedDeltaTime);
        Quaternion smoothedRotation = Quaternion.Slerp(transform.rotation, desiredTransform.GetR(), smoothSpeed * Time.fixedDeltaTime);
        transform.SetPositionAndRotation(smoothedPosition, smoothedRotation);
    }
}
