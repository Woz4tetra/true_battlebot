using UnityEngine;

public class Apriltag : MonoBehaviour {
    [SerializeField] private int tagId = 0;
    [SerializeField] Renderer tagRenderer;

    public int GetTagId() {
        return tagId;
    }

    public float GetSize() {
        return tagRenderer.bounds.size.x;
    }

    public Renderer GetRenderer() {
        return tagRenderer;
    }
}
