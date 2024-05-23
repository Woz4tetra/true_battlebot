using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using MathExtensions;
using System.Collections.Generic;

public abstract class BaseGameObjectSensor : MonoBehaviour
{
    protected TransformFrame frame;
    private Camera cameraView;
    private uint sentMessageCount = 0;
    private float rayCastOffset = 0.01f;  // Avoids raycast colliding with the camera
    protected ROSConnection ros;
    [SerializeField] private float maxDistance = 5.0f;
    [SerializeField] private LayerMask layerMask;
    [SerializeField] private bool debugRayCast = false;
    [SerializeField] private float publishRate = 0.0f;
    [SerializeField] private float viewAngleThreshold = 70.0f;
    [SerializeField] private string[] tagFilters = { };
    private float prevPublishTime = 0.0f;

    abstract protected void BaseGameObjectSensorStart();
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        frame = GetComponent<TransformFrame>();
        cameraView = GetComponent<Camera>();

        BaseGameObjectSensorStart();
    }

    void Update()
    {
        if (publishRate <= 0 || Time.unscaledTime - prevPublishTime > 1.0f / publishRate)
        {
            PublishTags();
            prevPublishTime = Time.unscaledTime;
        }
    }

    private void PublishTags()
    {
        List<GameObject> objs = new List<GameObject>();
        foreach (string tagFilter in tagFilters)
        {
            foreach (GameObject obj in GameObject.FindGameObjectsWithTag(tagFilter))
            {
                objs.Add(obj);
            }
        }
        VisibleTarget[] targets = ProcessObjects(objs.ToArray());
        TargetsCallback(targets);
    }

    abstract protected void TargetsCallback(VisibleTarget[] targets);

    protected void IncrementMessageCount()
    {
        sentMessageCount++;
    }
    protected uint GetMessageCount()
    {
        return sentMessageCount;
    }

    abstract protected VisibleTarget[] ProcessObjects(GameObject[] objs);

    public Matrix4x4 GetObjectPoseInCamera(Transform tagTransform)
    {
        Matrix4x4 tagMatrix = Matrix4x4.TRS(tagTransform.position, tagTransform.rotation, Vector3.one);
        Matrix4x4 thisMatrix = Matrix4x4.TRS(transform.position, transform.rotation, Vector3.one);
        Matrix4x4 cameraRotateMatrix = Matrix4x4.TRS(Vector3.zero, Quaternion.Euler(-90.0f, 0.0f, -90.0f), Vector3.one);
        Matrix4x4 relativeMatrix = thisMatrix.inverse * tagMatrix;
        relativeMatrix = cameraRotateMatrix * relativeMatrix;
        Vector3 relativePoint = relativeMatrix.GetT();
        Quaternion localRotation = relativeMatrix.GetR();
        return Matrix4x4.TRS(relativePoint, localRotation, Vector3.one);
    }

    protected bool IsVisible(GameObject obj)
    {
        Renderer renderer = obj.GetComponentInChildren<Renderer>();
        Plane[] planes = GeometryUtility.CalculateFrustumPlanes(cameraView);

        if (renderer == null)
        {
            Debug.LogWarning($"Object {obj.name} does not have a renderer");
            return false;
        }
        if (GeometryUtility.TestPlanesAABB(planes, renderer.bounds))
        {
            RaycastHit hit;
            Vector3 tagNormal = obj.transform.rotation * -Vector3.up;
            Vector3 cameraNormal = cameraView.transform.rotation * Vector3.forward;
            float tagAngle = Vector3.Angle(tagNormal, cameraNormal);
            if (tagAngle > viewAngleThreshold)
            {
                return false;
            }
            Vector3 directionVector = renderer.bounds.center - cameraView.transform.position;
            var measurementStart = rayCastOffset * directionVector + transform.position;
            var measurementRay = new Ray(measurementStart, directionVector.normalized);
            bool containsLayer = layerMask == (layerMask | (1 << obj.layer));
            if (!containsLayer)
            {
                return false;
            }
            if (Physics.Raycast(measurementRay, out hit, maxDistance, layerMask))
            {
                bool isUnObstructed = IsChild(ObjectUtils.GetTopLevelObject(hit.transform.gameObject), obj);
                if (debugRayCast)
                {
                    Debug.DrawRay(measurementStart, directionVector.normalized * hit.distance, isUnObstructed ? Color.green : Color.yellow);
                }
                return isUnObstructed;
            }
            return false;
        }
        else
        {
            return false;
        }
    }

    private bool IsChild(GameObject parent, GameObject check)
    {
        if (parent == check)
        {
            return true;
        }
        Transform child = null;
        for (int i = 0; i < parent.transform.childCount; i++)
        {
            child = parent.transform.GetChild(i);
            if (child.gameObject == check)
            {
                return true;
            }
            else
            {
                bool found = IsChild(child.gameObject, check);
                if (found)
                {
                    return true;
                }
            }
        }

        return false;
    }
}
