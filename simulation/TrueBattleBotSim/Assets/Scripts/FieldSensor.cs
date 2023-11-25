using System.Collections.Generic;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.VisualScripting;
using RosMessageTypes.BwInterfaces;
using RosMessageTypes.Std;
using UnityEngine;
using System;
using RosMessageTypes.Nav;

public class FieldSensor : BaseRectangleSensor
{
    [SerializeField] private string topic = "detections";
    Matrix4x4 fieldRotateMatrix = Matrix4x4.TRS(Vector3.zero, Quaternion.Euler(0.0f, 0.0f, 180.0f), Vector3.one);

    override protected void BaseRectangleSensorStart()
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
            Matrix4x4 targetPose = target.cameraRelativePose * fieldRotateMatrix;
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
                state = new OdometryMsg
                {
                    pose = new PoseWithCovarianceMsg {
                        pose = new PoseMsg {
                            position = targetPose.GetT().To<FLU>(),
                            orientation = targetPose.GetR().To<FLU>()
                        }
                    }
                },
                size = size
            };
            fields.Add(msg);
        }
        return fields.ToArray();
    }
}
