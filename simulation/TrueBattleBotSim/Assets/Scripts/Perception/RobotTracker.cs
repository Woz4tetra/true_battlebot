using System.Collections.Generic;
using UnityEngine;

public class RobotTracker : MonoBehaviour
{
    [SerializeField] List<ConfigurableKeypoint> keypoints;
    Bounds bounds;
    TransformFrame frame;
    Renderer trackerRenderer;
    Renderer[] childRenderers;
    string mainTag;

    void Start()
    {
        frame = ObjectUtils.GetComponentInTree<TransformFrame>(gameObject);
        mainTag = frame.gameObject.tag;
        trackerRenderer = gameObject.GetComponent<Renderer>();
        childRenderers = transform.GetComponentsInChildren<Renderer>();
    }

    public TransformFrame GetFrame()
    {
        return frame;
    }

    public string GetLabel()
    {
        return mainTag.ToLower();
    }

    public List<ConfigurableKeypoint> GetKeypoints()
    {
        return keypoints;
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
