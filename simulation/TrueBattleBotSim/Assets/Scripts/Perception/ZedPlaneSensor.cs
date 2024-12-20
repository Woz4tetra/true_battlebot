using System.Collections.Generic;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using MathExtensions;
using RosMessageTypes.Std;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class ZedPlaneSensor : BaseRectangleSensor
{
    [SerializeField] private string plane_request_topic = "plane_request";
    [SerializeField] private string plane_response_topic = "plane_response";
    private VisibleTarget[] prevTargets;
    Matrix4x4 fieldRotateMatrix = Matrix4x4.TRS(Vector3.zero, Quaternion.Euler(90.0f, 90.0f, 0.0f), Vector3.one);

    override protected void BaseGameObjectSensorStart()
    {
        RosTopicState responseTopicState = ros.GetTopic(plane_response_topic);
        if (responseTopicState == null)
        {
            ros.RegisterPublisher<PoseStampedMsg>(plane_response_topic);
        }
        ros.Subscribe<PointStampedMsg>(plane_request_topic, PlaneRequestCallback);
    }

    private void PlaneRequestCallback(PointStampedMsg pointMsg)
    {
        PoseStampedMsg[] msgs = ConvertTargetsToPoses(prevTargets);
        if (msgs.Length == 0)
        {
            Debug.LogWarning("No ZED plane targets found");
            return;
        }
        foreach (PoseStampedMsg msg in msgs)
        {
            ros.Publish(plane_response_topic, msg);
        }
    }

    override protected void TargetsCallback(VisibleTarget[] targets)
    {
        prevTargets = targets;
    }

    private PoseStampedMsg[] ConvertTargetsToPoses(VisibleTarget[] targets)
    {
        HeaderMsg header = new HeaderMsg
        {
            seq = GetMessageCount(),
            stamp = RosUtil.GetTimeMsg(),
            frame_id = frame.GetFrameId()
        };
        List<PoseStampedMsg> plane_poses = new List<PoseStampedMsg>();
        foreach (VisibleTarget target in targets)
        {
            Matrix4x4 targetPose = target.cameraRelativePose * fieldRotateMatrix;
            PointMsg point = targetPose.GetT().To<FLU>();
            QuaternionMsg orientation = targetPose.GetR().To<FLU>();
            PoseStampedMsg msg = new PoseStampedMsg
            {
                header = header,
                pose = new PoseMsg
                {
                    position = new PointMsg()
                    {
                        x = point.x,
                        y = point.y,
                        z = point.z
                    },
                    orientation = orientation
                }
            };
            plane_poses.Add(msg);
        }
        return plane_poses.ToArray();
    }
}
