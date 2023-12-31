using UnityEngine;

public class RectangleTarget : MonoBehaviour {
    [SerializeField] private int tagId = 0;
    [SerializeField] private string tagName = "";
    [SerializeField] Renderer tagRenderer;

    public int GetTagId() {
        return tagId;
    }

    public string GetTagName() {
        return tagName;
    }

    public Vector3 GetDimensions() {
        return tagRenderer.bounds.size;
    }

    public Renderer GetRenderer() {
        return tagRenderer;
    }

    public int GetLayer() {
        return tagRenderer.gameObject.layer;
    }
}
