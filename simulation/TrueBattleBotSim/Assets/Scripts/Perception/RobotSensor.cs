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


    override protected void PublishTargets()
    {
        RobotTracker[] robot_trackers = FindObjectsOfType<RobotTracker>();
        VisibleTarget[] targets = ProcessObjects(robot_trackers);
        EstimatedObjectArrayMsg msg = ConvertTargetsToRobots(targets);
        ros.Publish(topic, msg);
    }

    protected override void BaseGameObjectSensorStart()
    {
        ros.RegisterPublisher<EstimatedObjectArrayMsg>(topic);
    }

    private VisibleTarget[] ProcessObjects(RobotTracker[] robot_trackers)
    {
        List<VisibleTarget> targetList = new List<VisibleTarget>();
        foreach (RobotTracker robot in robot_trackers)
        {
            GameObject obj = robot.gameObject;
            if (!IsVisible(obj, robot.GetBounds()))
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
                dimensions = robot.GetBounds().size,
                cameraRelativePose = GetObjectPoseInCamera(obj.transform),
                frame = robot.GetFrame(),
                label = robot.GetLabel(),
                keypoints = robot.GetKeypoints().ToArray()
            };
            targetList.Add(tagMsg);
        }
        IncrementMessageCount();
        return targetList.ToArray();
    }

    private EstimatedObjectArrayMsg ConvertTargetsToRobots(VisibleTarget[] targets)
    {
        HeaderMsg header = new HeaderMsg
        {
            seq = GetMessageCount(),
            stamp = RosUtil.GetTimeMsg(),
            frame_id = frame.GetFrameId()
        };
        List<EstimatedObjectMsg> robots = new List<EstimatedObjectMsg>();
        foreach (VisibleTarget target in targets)
        {
            List<PoseMsg> keypoints = new List<PoseMsg>();
            List<string> keypoint_names = new List<string>();
            foreach (ConfigurableKeypoint keypoint in target.keypoints)
            {
                keypoints.Add(new PoseMsg
                {
                    position = keypoint.transform.position.To<FLU>(),
                    orientation = keypoint.transform.rotation.To<FLU>()
                });
                keypoint_names.Add(keypoint.name);
            }
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
                label = target.label,
                keypoints = keypoints.ToArray(),
                keypoint_names = keypoint_names.ToArray()
            };
            robots.Add(msg);
        }
        return new EstimatedObjectArrayMsg { robots = robots.ToArray() };
    }
}
