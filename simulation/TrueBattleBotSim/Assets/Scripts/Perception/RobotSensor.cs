using System.Collections.Generic;
using RosMessageTypes.BwInterfaces;
using RosMessageTypes.Std;
using UnityEngine;
using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using MathExtensions;
using Unity.Robotics.ROSTCPConnector;

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
        RosTopicState topicState = ros.GetTopic(topic);
        if (topicState == null || !topicState.IsPublisher)
        {
            ros.RegisterPublisher<EstimatedObjectArrayMsg>(topic);
        }
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

            Dictionary<string, Matrix4x4> keypointsInCamera = new Dictionary<string, Matrix4x4>();
            foreach (ConfigurableKeypoint keypoint in robot.GetKeypoints())
            {
                keypointsInCamera[keypoint.name] = GetObjectPoseInCamera(keypoint.transform);
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
                keypoints = keypointsInCamera
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
            Matrix4x4 tf_camera_from_robot = target.cameraRelativePose;
            foreach (KeyValuePair<string, Matrix4x4> keypointPair in target.keypoints)
            {
                Matrix4x4 tf_camera_from_keypoint = keypointPair.Value;
                Matrix4x4 tf_robot_from_keypoint = tf_camera_from_robot.inverse * tf_camera_from_keypoint;
                keypoints.Add(new PoseMsg
                {
                    position = tf_robot_from_keypoint.GetT().To<FLU>(),
                    orientation = tf_robot_from_keypoint.GetR().To<FLU>()
                });
                keypoint_names.Add(keypointPair.Key);
            }
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
                        position = tf_camera_from_robot.GetT().To<FLU>(),
                        orientation = tf_camera_from_robot.GetR().To<FLU>()
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
