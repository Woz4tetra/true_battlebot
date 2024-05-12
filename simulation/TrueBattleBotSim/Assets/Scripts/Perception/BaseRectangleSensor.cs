using UnityEngine;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using MathExtensions;
using RosMessageTypes.Std;

public abstract class BaseRectangleSensor : MonoBehaviour
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
    private float prevPublishTime = 0.0f;

    abstract protected void BaseRectangleSensorStart();
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        frame = GetComponent<TransformFrame>();
        cameraView = GetComponent<Camera>();

        BaseRectangleSensorStart();
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
        RectangleTarget[] tags = FindObjectsOfType<RectangleTarget>();
        VisibleTarget[] targets = ProcessTags(tags);
        TargetsCallback(targets);
    }

    abstract protected void TargetsCallback(VisibleTarget[] targets);

    protected uint GetMessageCount()
    {
        return sentMessageCount;
    }

    private VisibleTarget[] ProcessTags(RectangleTarget[] tags)
    {
        List<VisibleTarget> tagList = new List<VisibleTarget>();
        foreach (RectangleTarget tag in tags)
        {
            if (!IsVisible(tag))
            {
                continue;
            }
            VisibleTarget tagMsg = new VisibleTarget
            {
                header = new HeaderMsg
                {
                    seq = sentMessageCount,
                    stamp = RosUtil.GetTimeMsg(),
                    frame_id = frame.GetFrameId()
                },
                tagId = tag.GetTagId(),
                dimensions = tag.GetDimensions(),
                cameraRelativePose = GetTagPose(tag.transform)
            };
            tagList.Add(tagMsg);
        }
        sentMessageCount++;
        return tagList.ToArray();
    }

    public Matrix4x4 GetTagPose(Transform tagTransform)
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

    private bool IsVisible(RectangleTarget tag)
    {
        Renderer tagRenderer = tag.GetRenderer();
        Plane[] planes = GeometryUtility.CalculateFrustumPlanes(cameraView);

        if (tagRenderer == null)
        {
            Debug.LogWarning($"Tag {tag.name} does not have a renderer");
            return false;
        }
        if (GeometryUtility.TestPlanesAABB(planes, tagRenderer.bounds))
        {
            RaycastHit hit;
            Vector3 tagNormal = tag.transform.rotation * -Vector3.up;
            Vector3 cameraNormal = cameraView.transform.rotation * Vector3.forward;
            float tagAngle = Vector3.Angle(tagNormal, cameraNormal);
            if (tagAngle > viewAngleThreshold)
            {
                return false;
            }
            Vector3 directionVector = tagRenderer.bounds.center - cameraView.transform.position;
            var measurementStart = rayCastOffset * directionVector + transform.position;
            var measurementRay = new Ray(measurementStart, directionVector.normalized);
            bool containsLayer = layerMask == (layerMask | (1 << tag.GetLayer()));
            if (!containsLayer)
            {
                return false;
            }
            if (Physics.Raycast(measurementRay, out hit, maxDistance, layerMask))
            {
                bool isUnObstructed = IsChild(ObjectUtils.GetTopLevelObject(hit.transform.gameObject), tag.gameObject);
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
