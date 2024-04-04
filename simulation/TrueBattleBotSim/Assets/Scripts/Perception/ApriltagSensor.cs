using RosMessageTypes.ApriltagRos;
using System.Collections.Generic;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using MathExtensions;
using RosMessageTypes.Std;
using UnityEngine;

public class ApriltagSensor : BaseRectangleSensor
{
    [SerializeField] private string topic = "detections";
    override protected void BaseRectangleSensorStart()
    {
        ros.RegisterPublisher<AprilTagDetectionArrayMsg>(topic);
    }

    override protected void TargetsCallback(VisibleTarget[] targets)
    {
        AprilTagDetectionArrayMsg msg = ConvertTargetsToApriltags(targets);
        ros.Publish(topic, msg);
    }

    private AprilTagDetectionArrayMsg ConvertTargetsToApriltags(VisibleTarget[] targets)
    {
        HeaderMsg header = new HeaderMsg
        {
            seq = GetMessageCount(),
            stamp = RosUtil.GetTimeMsg(),
            frame_id = frame.GetFrameId()
        };
        AprilTagDetectionArrayMsg tagArrayMsg = new AprilTagDetectionArrayMsg
        {
            header = header
        };
        List<AprilTagDetectionMsg> tagList = new List<AprilTagDetectionMsg>();
        foreach (VisibleTarget target in targets)
        {
            AprilTagDetectionMsg tagMsg = new AprilTagDetectionMsg
            {
                id = new int[] { target.tagId },
                size = new double[] { Mathf.Max(target.dimensions.x, target.dimensions.y, target.dimensions.z) },
                pose = {
                    header = header,
                    pose = {
                        pose = new PoseMsg
                        {
                            position = target.cameraRelativePose.GetT().To<FLU>(),
                            orientation = target.cameraRelativePose.GetR().To<FLU>()
                        }
                    }
                }
            };
            tagList.Add(tagMsg);
        }
        tagArrayMsg.detections = tagList.ToArray();
        return tagArrayMsg;
    }
}
