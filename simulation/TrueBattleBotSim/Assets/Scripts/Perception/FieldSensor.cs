using System.Collections.Generic;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using MathExtensions;
using RosMessageTypes.BwInterfaces;
using RosMessageTypes.Std;
using UnityEngine;
using System;

public class FieldSensor : BaseRectangleSensor
{
    [SerializeField] private string topic = "detections";
    [SerializeField] private string child_frame_id = "map_relative";
    [SerializeField] private string label = "field";

    override protected void BaseGameObjectSensorStart()
    {
        ros.RegisterPublisher<EstimatedObjectMsg>(topic);
    }

    override protected void TargetsCallback(VisibleTarget[] targets)
    {
        EstimatedObjectMsg[] msgs = ConvertTargetsToFields(targets);
        foreach (EstimatedObjectMsg msg in msgs)
        {
            ros.Publish(topic, msg);
        }
    }

    private EstimatedObjectMsg[] ConvertTargetsToFields(VisibleTarget[] targets)
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
                child_frame_id = child_frame_id,
                size = size,
                label = label
            };
            fields.Add(msg);
        }
        return fields.ToArray();
    }
}
