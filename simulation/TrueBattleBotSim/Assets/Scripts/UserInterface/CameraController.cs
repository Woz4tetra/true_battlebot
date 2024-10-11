using MathExtensions;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class CameraController : MonoBehaviour
{
    [SerializeField] Matrix4x4 focusObject = Matrix4x4.identity;
    [SerializeField] float smoothSpeed = 20.0f;
    [SerializeField] float azimuthScale = 5.0f;
    [SerializeField] float elevationScale = 2.0f;
    [SerializeField] float distanceSpeed = 30.0f;
    [SerializeField] SerializableTuple<float, float> elevationLimits = new SerializableTuple<float, float>(-Mathf.PI / 4, Mathf.PI / 4);
    [SerializeField] SerializableTuple<float, float> distanceLimits = new SerializableTuple<float, float>(1.0f, 5.0f);
    [SerializeField] LayerMask layerMask;
    float azimuthAngle = 0.0f, elevationAngle = 0.0f, distance = 10.0f;
    float clickedAzimuthAngle = 0.0f, clickedElevationAngle = 0.0f;
    private Vector2 prevClickPosition = Vector2.zero;

    private bool prevButtonState = false;
    private bool enableControls = true;
    private static Quaternion QuaternionLookRotation(Vector3 forward, Vector3 up)
    {
        forward.Normalize();

        Vector3 vector = Vector3.Normalize(forward);
        Vector3 vector2 = Vector3.Normalize(Vector3.Cross(up, vector));
        Vector3 vector3 = Vector3.Cross(vector, vector2);
        var m00 = vector2.x;
        var m01 = vector2.y;
        var m02 = vector2.z;
        var m10 = vector3.x;
        var m11 = vector3.y;
        var m12 = vector3.z;
        var m20 = vector.x;
        var m21 = vector.y;
        var m22 = vector.z;


        float num8 = (m00 + m11) + m22;
        var quaternion = new Quaternion();
        if (num8 > 0f)
        {
            var num = (float)Mathf.Sqrt(num8 + 1f);
            quaternion.w = num * 0.5f;
            num = 0.5f / num;
            quaternion.x = (m12 - m21) * num;
            quaternion.y = (m20 - m02) * num;
            quaternion.z = (m01 - m10) * num;
            return quaternion;
        }
        if ((m00 >= m11) && (m00 >= m22))
        {
            var num7 = (float)Mathf.Sqrt(((1f + m00) - m11) - m22);
            var num4 = 0.5f / num7;
            quaternion.x = 0.5f * num7;
            quaternion.y = (m01 + m10) * num4;
            quaternion.z = (m02 + m20) * num4;
            quaternion.w = (m12 - m21) * num4;
            return quaternion;
        }
        if (m11 > m22)
        {
            var num6 = (float)Mathf.Sqrt(((1f + m11) - m00) - m22);
            var num3 = 0.5f / num6;
            quaternion.x = (m10 + m01) * num3;
            quaternion.y = 0.5f * num6;
            quaternion.z = (m21 + m12) * num3;
            quaternion.w = (m20 - m02) * num3;
            return quaternion;
        }
        var num5 = (float)Mathf.Sqrt(((1f + m22) - m00) - m11);
        var num2 = 0.5f / num5;
        quaternion.x = (m20 + m02) * num2;
        quaternion.y = (m21 + m12) * num2;
        quaternion.z = 0.5f * num5;
        quaternion.w = (m01 - m10) * num2;
        return quaternion;
    }
    void initializePolarCoordinates()
    {
        Vector3 direction = transform.position - focusObject.GetT();
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
        Quaternion rotation = Quaternion.LookRotation(-position);
        return Matrix4x4.TRS(position + focusObject.GetT(), rotation, Vector3.one);
    }

    void Start()
    {
        if (focusObject == null)
        {
            focusObject = Matrix4x4.identity;
        }
        initializePolarCoordinates();
    }

    private bool GetRightMouseDown()
    {
        return Input.GetButton("Fire2");
    }

    private bool DidMiddleMouseUp()
    {
        return Input.GetMouseButtonUp(2);
    }

    private bool RaycastToFocusObject(out Matrix4x4 newFocusObject)
    {
        newFocusObject = Matrix4x4.identity;
        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
        RaycastHit hit;
        if (Physics.Raycast(ray, out hit, Mathf.Infinity, layerMask))
        {
            newFocusObject = Matrix4x4.TRS(hit.point, Quaternion.identity, Vector3.one);
            return true;
        }
        return false;
    }

    public void EnableControls(bool enable)
    {
        enableControls = enable;
    }

    private Vector2 GetMovementVector()
    {
        bool buttonState = GetRightMouseDown();
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

        if (DidMiddleMouseUp())
        {
            Matrix4x4 newFocusObject;
            if (RaycastToFocusObject(out newFocusObject))
            {
                focusObject = newFocusObject;
            }
        }
    }
}
