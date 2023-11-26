using System.Collections.Generic;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.VisualScripting;
using RosMessageTypes.Std;
using RosMessageTypes.ZedInterfaces;
using UnityEngine;
using System;

public class ZedPlaneSensor : BaseRectangleSensor {
    [SerializeField] private string plane_request_topic = "plane_request";
    [SerializeField] private string plane_response_topic = "plane_response";
    private VisibleTarget[] prevTargets;
    Matrix4x4 fieldRotateMatrix = Matrix4x4.TRS(Vector3.zero, Quaternion.Euler(90.0f, 0.0f, 0.0f), Vector3.one);

    override protected void BaseRectangleSensorStart()
    {
        ros.RegisterPublisher<PlaneStampedMsg>(plane_response_topic);
        ros.Subscribe<PointStampedMsg>(plane_request_topic, PlaneRequestCallback);
    }

    private void PlaneRequestCallback(PointStampedMsg pointMsg) {
        PlaneStampedMsg[] msgs = ConvertTargetsToPlanes(prevTargets);
        foreach (PlaneStampedMsg msg in msgs)
        {
            ros.Publish(plane_response_topic, msg);
        }
    }

    override protected void TargetsCallback(VisibleTarget[] targets) {
        prevTargets = targets;
    }

    private PlaneStampedMsg[] ConvertTargetsToPlanes(VisibleTarget[] targets) {
        HeaderMsg header = new HeaderMsg {
            seq = GetMessageCount(),
            stamp = RosUtil.GetTimeMsg(),
            frame_id = frame.GetFrameId()
        };
        List<PlaneStampedMsg> planes = new List<PlaneStampedMsg>();
        foreach (VisibleTarget target in targets)
        {
            Matrix4x4 targetPose = target.cameraRelativePose * fieldRotateMatrix;
            PointMsg point = targetPose.GetT().To<FLU>();
            QuaternionMsg orientation = targetPose.GetR().To<FLU>();
            Vector3Msg size = target.dimensions.To<FLU>();
            size = new Vector3Msg {
                x = Math.Abs(size.x),
                y = Math.Abs(size.y),
                z = Math.Abs(size.z)
            };
            PlaneStampedMsg msg = new PlaneStampedMsg {
                header = header,
                pose = new TransformMsg {
                    translation = new Vector3Msg() {
                        x = point.x,
                        y = point.y,
                        z = point.z
                    },
                    rotation = orientation
                },
                extents = new float[2] {
                    (float)size.x,
                    (float)size.y,
                }
            };
            planes.Add(msg);
        }
        return planes.ToArray();
    }
}
