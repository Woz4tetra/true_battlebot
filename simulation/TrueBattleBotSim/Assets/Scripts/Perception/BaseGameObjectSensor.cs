using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using MathExtensions;
using System.Collections.Generic;

[RequireComponent(typeof(TransformFrame))]
public abstract class BaseGameObjectSensor : MonoBehaviour
{
    public class TrackedDebugRayCast
    {
        private Vector3 debugRayCastStart;
        private Vector3 debugRayCastDirection;
        private Color debugRayCastColor;

        public TrackedDebugRayCast(Vector3 start, Vector3 direction, Color color)
        {
            debugRayCastStart = start;
            debugRayCastDirection = direction;
            debugRayCastColor = color;
        }

        public void Draw()
        {
            Debug.DrawRay(debugRayCastStart, debugRayCastDirection, debugRayCastColor);
        }
    }

    [SerializeField] protected TransformFrame frame;
    [SerializeField] private Camera cameraView;
    private uint sentMessageCount = 0;
    private float rayCastOffset = 0.01f;  // Avoids raycast colliding with the camera
    protected ROSConnection ros;
    [SerializeField] private float maxDistance = 5.0f;
    [SerializeField] private LayerMask layerMask;
    [SerializeField] private bool debugRayCast = false;
    [SerializeField] private float publishRate = 0.0f;
    [SerializeField] private float viewAngleThreshold = 70.0f;
    [SerializeField] private bool debugVisibilityReason = false;
    private float prevPublishTime = 0.0f;
    [SerializeField] private bool useCameraView = true;

    private Dictionary<string, TrackedDebugRayCast> debugRayCasts = new Dictionary<string, TrackedDebugRayCast>();

    abstract protected void BaseGameObjectSensorStart();
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        frame = ObjectUtils.GetComponentInTree<TransformFrame>(gameObject);
        cameraView = GetComponent<Camera>();
        if (cameraView == null)
        {
            Debug.LogWarning($"No camera found on {gameObject.name}. Always returning true for visibility.");
            useCameraView = false;
        }

        BaseGameObjectSensorStart();
    }

    void Update()
    {
        if (debugRayCast)
        {
            foreach (var entry in debugRayCasts)
            {
                entry.Value.Draw();
            }
        }
        if (publishRate <= 0 || Time.unscaledTime - prevPublishTime > 1.0f / publishRate)
        {
            PublishTargets();
            prevPublishTime = Time.unscaledTime;
        }
    }

    abstract protected void PublishTargets();


    protected void IncrementMessageCount()
    {
        sentMessageCount++;
    }
    protected uint GetMessageCount()
    {
        return sentMessageCount;
    }


    public Matrix4x4 GetObjectPoseInCamera(Transform tfWorldFromObject)
    {
        Matrix4x4 matWorldFromObject = Matrix4x4.TRS(tfWorldFromObject.position, tfWorldFromObject.rotation, Vector3.one);
        Matrix4x4 matWorldFromCameraRotated = Matrix4x4.TRS(transform.position, transform.rotation, Vector3.one);
        Matrix4x4 matCameraFromCameraRotated = Matrix4x4.TRS(Vector3.zero, Quaternion.Euler(-90.0f, 0.0f, -90.0f), Vector3.one);
        Matrix4x4 matCameraRotatedFromWorld = matWorldFromCameraRotated.inverse;
        Matrix4x4 matCameraFromObject = matCameraFromCameraRotated * (matCameraRotatedFromWorld * matWorldFromObject);
        Vector3 posCameraToObject = matCameraFromObject.GetT();
        Quaternion quatCameraToObject = matCameraFromObject.GetR();
        return Matrix4x4.TRS(posCameraToObject, quatCameraToObject, Vector3.one);
    }

    protected bool IsVisible(GameObject obj, Bounds bounds)
    {
        if (!useCameraView)
        {
            return true;
        }
        Plane[] planes = GeometryUtility.CalculateFrustumPlanes(cameraView);
        if (!GeometryUtility.TestPlanesAABB(planes, bounds))
        {
            if (debugVisibilityReason) Debug.Log($"Object {obj.name} is not in camera view");
            return false;
        }
        RaycastHit hit;
        Vector3 tagNormal = obj.transform.rotation * -Vector3.up;
        Vector3 cameraNormal = cameraView.transform.rotation * Vector3.forward;
        float tagAngle = Vector3.Angle(tagNormal, cameraNormal);
        if (tagAngle > viewAngleThreshold)
        {
            if (debugVisibilityReason) Debug.Log($"Object {obj.name} is outside view angle threshold. Angle: {tagAngle} Threshold: {viewAngleThreshold}");
            return false;
        }
        Vector3 directionVector = obj.transform.position - cameraView.transform.position;
        var measurementStart = rayCastOffset * directionVector + transform.position;
        var measurementRay = new Ray(measurementStart, directionVector.normalized);
        bool containsLayer = layerMask == (layerMask | (1 << obj.layer));
        if (!containsLayer)
        {
            if (debugVisibilityReason) Debug.Log($"Object {obj.name} is not in the layer mask. Layer name: {LayerMask.LayerToName(obj.layer)}");
            return false;
        }
        if (!Physics.Raycast(measurementRay, out hit, maxDistance, layerMask) && hit.transform != null)
        {
            if (debugVisibilityReason) Debug.Log($"Object {obj.name} is obstructed by {hit.transform.gameObject.name}");
            return false;
        }
        if (hit.transform == null)
        {
            return true;
        }
        GameObject toplevelObj = ObjectUtils.GetTopLevelObject(hit.transform.gameObject);
        bool isUnObstructed = ObjectUtils.IsChild(toplevelObj, obj);
        if (debugRayCast)
        {
            debugRayCasts[obj.name] = new TrackedDebugRayCast(
                measurementStart,
                directionVector.normalized * hit.distance,
                isUnObstructed ? Color.green : Color.yellow
            );
        }
        return isUnObstructed;
    }
}
