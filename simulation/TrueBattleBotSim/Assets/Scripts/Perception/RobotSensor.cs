using System.Collections.Generic;
using RosMessageTypes.BwInterfaces;
using RosMessageTypes.Std;
using UnityEngine;
using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using MathExtensions;

public class RobotSensor : BaseGameObjectSensor
{
    [SerializeField] private string topic = "ground_truth/robots";

    protected override void BaseGameObjectSensorStart()
    {
        ros.RegisterPublisher<EstimatedObjectArrayMsg>(topic);
    }

    override protected VisibleTarget[] ProcessObjects(GameObject[] objs)
    {
        List<VisibleTarget> targetList = new List<VisibleTarget>();
        foreach (GameObject obj in objs)
        {
            if (!IsVisible(obj))
            {
                continue;
            }
            VisibleTarget tagMsg = new VisibleTarget
            {
                header = new HeaderMsg
                {
                    seq = GetMessageCount(),
                    stamp = RosUtil.GetTimeMsg(),
                    frame_id = frame.GetFrameId()
                },
                objectId = 0,
                dimensions = obj.GetComponent<Renderer>().bounds.size,
                cameraRelativePose = GetObjectPoseInCamera(obj.transform),
                frame = obj.GetComponent<TransformFrame>()
            };
            targetList.Add(tagMsg);
        }
        IncrementMessageCount();
        return targetList.ToArray();
    }

    protected override void TargetsCallback(VisibleTarget[] targets)
    {
        EstimatedObjectMsg[] msgs = ConvertTargetsToRobots(targets);
        foreach (EstimatedObjectMsg msg in msgs)
        {
            ros.Publish(topic, msg);
        }
    }


    private EstimatedObjectMsg[] ConvertTargetsToRobots(VisibleTarget[] targets)
    {
        HeaderMsg header = new HeaderMsg
        {
            seq = GetMessageCount(),
            stamp = RosUtil.GetTimeMsg(),
            frame_id = frame.GetFrameId()
        };
        List<EstimatedObjectMsg> fields = new List<EstimatedObjectMsg>();
        foreach (VisibleTarget target in targets)
        {
            Matrix4x4 targetPose = target.cameraRelativePose;
            Vector3Msg size = target.dimensions.To<FLU>();
            size = new Vector3Msg
            {
                x = Math.Abs(size.x),
                y = Math.Abs(size.y),
                z = Math.Abs(size.z)
            };
            EstimatedObjectMsg msg = new EstimatedObjectMsg
            {

                header = header,
                pose = new PoseWithCovarianceMsg
                {
                    pose = new PoseMsg
                    {
                        position = targetPose.GetT().To<FLU>(),
                        orientation = targetPose.GetR().To<FLU>()
                    }
                },
                child_frame_id = target.frame.GetFrameId(),
                size = size,
                label = target.frame.GetFrameId()
            };
            fields.Add(msg);
        }
        return fields.ToArray();
    }
}
