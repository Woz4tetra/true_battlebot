using UnityEngine;

public class RectangleTarget : MonoBehaviour
{
    [SerializeField] private int tagId = 0;
    Renderer targetRenderer;

    void Start()
    {
        targetRenderer = GetComponentInChildren<Renderer>();
    }

    public int GetTagId()
    {
        return tagId;
    }

    public Bounds GetBounds()
    {
        return targetRenderer.bounds;
    }
}
