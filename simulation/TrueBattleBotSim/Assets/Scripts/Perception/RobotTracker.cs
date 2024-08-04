using UnityEngine;

public class RobotTracker : MonoBehaviour
{
    Bounds bounds;
    TransformFrame frame;
    Renderer trackerRenderer;
    Renderer[] childRenderers;

    void Start()
    {
        frame = ObjectUtils.GetComponentInTree<TransformFrame>(gameObject);
        trackerRenderer = gameObject.GetComponent<Renderer>();
        childRenderers = transform.GetComponentsInChildren<Renderer>();
    }

    public TransformFrame GetFrame()
    {
        return frame;
    }

    public Bounds GetBounds()
    {
        if (trackerRenderer != null)
        {
            bounds = trackerRenderer.bounds;
        }
        else if (childRenderers != null && childRenderers.Length != 0)
        {
            bounds = childRenderers[0].bounds;
        }
        else
        {
            bounds = new Bounds();
        }
        if (childRenderers != null)
        {
            foreach (Renderer render in childRenderers)
            {
                bounds.Encapsulate(render.bounds);
            }
        }
        return bounds;
    }
}
