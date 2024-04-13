using UnityEngine;

public class ArrowIndicator : MonoBehaviour
{
    public void Set2D(float x, float y, float z, float yaw, float scale = 1.0f)
    {
        transform.position = new Vector3(x, y, z);
        transform.rotation = Quaternion.Euler(0, yaw, 0);
        transform.localScale = new Vector3(1.0f, 1.0f, scale);
    }

    public void SetColor(Color color)
    {
        foreach (Renderer renderer in GetComponentsInChildren<Renderer>())
        {
            renderer.material.color = color;
        }
    }
}