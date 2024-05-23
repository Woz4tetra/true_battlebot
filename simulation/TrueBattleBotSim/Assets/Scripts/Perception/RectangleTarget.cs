using UnityEngine;

public class RectangleTarget : MonoBehaviour
{
    [SerializeField] private int tagId = 0;
    [SerializeField] Renderer tagRenderer;

    void Start()
    {
        if (tagRenderer == null)
        {
            tagRenderer = GetComponent<Renderer>();
        }
    }

    public int GetTagId()
    {
        return tagId;
    }

    public Vector3 GetDimensions()
    {
        return tagRenderer.bounds.size;
    }

    public Renderer GetRenderer()
    {
        return tagRenderer;
    }

    public int GetLayer()
    {
        return tagRenderer.gameObject.layer;
    }
}
