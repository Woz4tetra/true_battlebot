using MathExtensions;
using UnityEngine;

public class ArrowIndicator : MonoBehaviour
{
    public void SetPose(Matrix4x4 desiredPose)
    {
        Vector3 desiredPosition = desiredPose.GetT();
        transform.position = new Vector3(desiredPosition.x, desiredPosition.y + 0.05f, desiredPosition.z);
        transform.rotation = desiredPose.GetR();
        transform.localScale = desiredPose.GetS();
    }

    public void SetColor(Color color)
    {
        foreach (Renderer renderer in GetComponentsInChildren<Renderer>())
        {
            renderer.material.color = color;
        }
    }
}