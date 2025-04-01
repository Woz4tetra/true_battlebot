using System;
using MathExtensions;
using RosMessageTypes.Geometry;
using UnityEngine;


public class PIDFollowerEngine : BaseFollowerEngine
{
    [SerializeField] float matchAngleDistanceThreshold = 0.01f;
    [SerializeField] PidConfig linearPIDConfig = new PidConfig(2.0f, 0.0f, 0.0f, 1.0f);
    [SerializeField] PidConfig angularPIDConfig = new PidConfig(10.0f, 0.1f, 1.0f, 1.0f);

    bool feedforwardGoalVelocity = false;

    PID linearPID, angularPID;

    public PIDFollowerEngine() : base()
    {
        linearPID = new PID(linearPIDConfig);
        angularPID = new PID(angularPIDConfig);
    }

    public virtual void SetFeedforwardGoalVelocity(bool feedforward)
    {
        feedforwardGoalVelocity = feedforward;
    }

    public void SetLinearPIDConfig(PidConfig config)
    {
        linearPID.SetConfig(config);
    }

    public void SetAngularPIDConfig(PidConfig config)
    {
        angularPID.SetConfig(config);
    }

    public override void Reset()
    {
        linearPID.Reset();
        angularPID.Reset();
        feedforwardGoalVelocity = false;
    }

    public override TwistMsg ComputeVelocity(Matrix4x4 currentPose, Matrix4x4 goalPose, Velocity2d currentVelocity, Velocity2d goalVelocity)
    {
        Matrix4x4 relativePose = currentPose.inverse * goalPose;
        Vector3 relativePosition = relativePose.GetT();
        bool isGoalBehind = relativePosition.x < 0.0f;

        if (Time.deltaTime <= 0.0f)
        {
            return new TwistMsg();
        }

        float linearVelocity = linearPID.Update(relativePosition.x, 0.0f, Time.deltaTime);
        float angularVelocity;
        if (relativePosition.magnitude > matchAngleDistanceThreshold)
        {
            float heading = Mathf.Atan2(relativePosition.y, relativePosition.x);
            if (isGoalBehind)
            {
                heading += Mathf.PI;
            }
            heading = MathfEx.NormalizeAnglePi(heading);
            angularVelocity = angularPID.Update(heading, 0.0f, Time.deltaTime);
        }
        else
        {
            float relativeAngle = relativePose.GetR().eulerAngles.z * Mathf.Deg2Rad;
            relativeAngle = MathfEx.NormalizeAnglePi(relativeAngle);
            angularVelocity = angularPID.Update(relativeAngle, 0.0f, Time.deltaTime);
        }

        if (feedforwardGoalVelocity)
        {
            linearVelocity += goalVelocity.vx;
            angularVelocity += goalVelocity.vyaw;
        }
        else
        {
            float maxLinearSpeedMagnitude = Mathf.Abs(goalVelocity.vx);
            linearVelocity = Mathf.Clamp(linearVelocity, -maxLinearSpeedMagnitude, maxLinearSpeedMagnitude);
            if (goalVelocity.vx < 0)
            {
                linearVelocity *= -1;
            }
            float maxAngularSpeedMagnitude = Mathf.Abs(goalVelocity.vyaw);
            angularVelocity = Mathf.Clamp(angularVelocity, -maxAngularSpeedMagnitude, maxAngularSpeedMagnitude);
            if (goalVelocity.vyaw < 0)
            {
                angularVelocity *= -1;
            }
        }
        return new TwistMsg
        {
            linear = new Vector3Msg { x = linearVelocity },
            angular = new Vector3Msg { z = angularVelocity }
        };
    }
}