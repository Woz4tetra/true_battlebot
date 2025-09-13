using UnityEngine;

[RequireComponent(typeof(ArticulationBody))]
public class PivotingLifter : MonoBehaviour
{
    ArticulationBody pivotBody;

    void Start()
    {
        pivotBody = GetComponent<ArticulationBody>();
    }

    public void SetAngle(float angleDegrees)
    {
        ArticulationDrive drive = pivotBody.xDrive;
        drive.target = angleDegrees;
        pivotBody.xDrive = drive;
    }
}
