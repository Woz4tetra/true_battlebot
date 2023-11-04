using System.Collections.Generic;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.VisualScripting;
using RosMessageTypes.BwInterfaces;
using RosMessageTypes.Std;
using UnityEngine;
using System;

public class FieldSensor : BaseRectangleSensor {
    [SerializeField] private string topic = "detections";
    override protected void BaseRectangleSensorStart()
    {
        ros.RegisterPublisher<EstimatedFieldMsg>(topic);
    }

    override protected void TargetsCallback(VisibleTarget[] targets) {
        EstimatedFieldMsg[] msgs = ConvertTargetsToFields(targets);
        foreach (EstimatedFieldMsg msg in msgs)
        {
            ros.Publish(topic, msg);
        }
    }

    private EstimatedFieldMsg[] ConvertTargetsToFields(VisibleTarget[] targets) {
        HeaderMsg header = new HeaderMsg {
            seq = GetMessageCount(),
            stamp = RosUtil.GetTimeMsg(),
            frame_id = frame.GetFrameId()
        };
        List<EstimatedFieldMsg> fields = new List<EstimatedFieldMsg>();
        foreach (VisibleTarget target in targets)
        {
            Vector3Msg size = target.dimensions.To<FLU>();
            size = new Vector3Msg {
                x = Math.Abs(size.x),
                y = Math.Abs(size.y),
                z = Math.Abs(size.z)
            };
            EstimatedFieldMsg msg = new EstimatedFieldMsg {
                header = header,
                center = new PoseMsg {
                    position = target.cameraRelativePose.GetT().To<FLU>(),
                    orientation = target.cameraRelativePose.GetR().To<FLU>()
                },
                size = size
            };
            fields.Add(msg);
        }
        return fields.ToArray();
    }
}
