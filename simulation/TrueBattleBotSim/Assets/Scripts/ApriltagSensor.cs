using UnityEngine;
using RosMessageTypes.ApriltagRos;
using System.Collections.Generic;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector;
using Unity.VisualScripting;

public class ApriltagSensor : MonoBehaviour {
    private TransformFrame frame;
    private Camera cameraView;
    private uint sentMessageCount = 0;
    private float rayCastOffset = 0.01f;  // Avoids raycast colliding with the camera
    private ROSConnection ros;
    [SerializeField] private float maxDistance = 5.0f;
    [SerializeField] private LayerMask layerMask;
    [SerializeField] private bool debugRayCast = false;
    [SerializeField] private string topic = "tag_detections";
    [SerializeField] private float publishRate = 0.0f;
    private float publishStartDelay = 1.0f;
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        frame = GetComponent<TransformFrame>();
        cameraView = GetComponent<Camera>();
        ros.RegisterPublisher<AprilTagDetectionArrayMsg>(topic);

        if (publishRate > 0) {
            InvokeRepeating("PublishTags", publishStartDelay, 1.0f / publishRate);
        }
    }

    void Update()
    {
        if (publishRate <= 0) {
            PublishTags();
        }
    }

    private void PublishTags() {
        Apriltag[] tags = FindObjectsOfType<Apriltag>();
        AprilTagDetectionArrayMsg msg = GetAprilTagArrayMsg(tags);
        ros.Publish(topic, msg);
    }

    private AprilTagDetectionArrayMsg GetAprilTagArrayMsg(Apriltag[] tags)
    {
        AprilTagDetectionArrayMsg tagArrayMsg = new AprilTagDetectionArrayMsg
        {
            header = {
                seq = sentMessageCount,
                stamp = RosUtil.GetTimeMsg(),
                frame_id = frame.GetFrameId()
            }
        };
        List<AprilTagDetectionMsg> tagList = new List<AprilTagDetectionMsg>();
        foreach (Apriltag tag in tags)
        {
            if (!IsVisible(tag))
            {
                continue;
            }
            AprilTagDetectionMsg tagMsg = new AprilTagDetectionMsg
            {
                id = new int[] { tag.GetTagId() },
                size = new double[] { tag.GetSize() },
                pose = {
                    header = {
                        seq = sentMessageCount,
                        stamp = RosUtil.GetTimeMsg(),
                        frame_id = frame.GetFrameId()
                    },
                    pose = {
                        pose = GetTagPose(tag.transform)
                    }
                }
            };
            tagList.Add(tagMsg);
        }
        tagArrayMsg.detections = tagList.ToArray();
        sentMessageCount++;
        return tagArrayMsg;
    }

    public PoseMsg GetTagPose(Transform tagTransform) {
        Matrix4x4 tagMatrix = Matrix4x4.TRS(tagTransform.position, tagTransform.rotation, Vector3.one);
        Matrix4x4 thisMatrix = Matrix4x4.TRS(transform.position, transform.rotation, Vector3.one);
        Matrix4x4 cameraRotateMatrix = Matrix4x4.TRS(Vector3.zero, Quaternion.Euler(-90.0f, 0.0f, -90.0f), Vector3.one);
        Matrix4x4 relativeMatrix = thisMatrix.inverse * tagMatrix;
        relativeMatrix = cameraRotateMatrix * relativeMatrix;
        // Quaternion localRotation = Quaternion.Inverse(transform.rotation) * tagTransform.rotation;
        Vector3 relativePoint = relativeMatrix.GetT();
        Quaternion localRotation = relativeMatrix.GetR();
        return new PoseMsg
        {
            position = relativePoint.To<FLU>(),
            orientation = localRotation.To<FLU>()
        };
    }

    private bool IsVisible(Apriltag tag)
    {
        Renderer tagRenderer = tag.GetRenderer();
        Plane[] planes = GeometryUtility.CalculateFrustumPlanes(cameraView);

        if (GeometryUtility.TestPlanesAABB(planes, tagRenderer.bounds))
        {
            RaycastHit hit;
            Vector3 directionVector = tagRenderer.bounds.center - cameraView.transform.position;
            var measurementStart = rayCastOffset * directionVector + transform.position;
            var measurementRay = new Ray(measurementStart, directionVector.normalized);
            if (Physics.Raycast(measurementRay, out hit, maxDistance, layerMask))
            {
                bool isUnObstructed = IsChild(GetTopLevelObject(hit.transform.gameObject), tag.gameObject);
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

    private GameObject GetTopLevelObject(GameObject obj)
    {
        Transform tf = obj.transform;
        while (true)
        {
            if (tf.parent == null)
            {
                break;
            }
            tf = tf.parent;
        }
        return tf.gameObject;
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
