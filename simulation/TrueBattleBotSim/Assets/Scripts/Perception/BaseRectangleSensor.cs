using UnityEngine;
using System.Collections.Generic;
using RosMessageTypes.Std;

public abstract class BaseRectangleSensor : BaseGameObjectSensor
{
    override protected void PublishTargets()
    {
        RectangleTarget[] tags = FindObjectsByType<RectangleTarget>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);
        VisibleTarget[] targets = ProcessObjects(tags);
        TargetsCallback(targets);
    }

    abstract protected void TargetsCallback(VisibleTarget[] targets);

    private VisibleTarget[] ProcessObjects(RectangleTarget[] tags)
    {
        List<VisibleTarget> tagList = new List<VisibleTarget>();
        foreach (RectangleTarget tag in tags)
        {
            if (tag == null || !IsVisible(tag.gameObject, tag.GetBounds()))
            {
                continue;
            }
            VisibleTarget tagMsg = new VisibleTarget
            {
                header = new HeaderMsg
                {
                    seq = GetMessageCount(),
                    stamp = RosUtil.GetTimeMsg(),
                    frame_id = frame.GetFrameId()
                },
                objectId = tag.GetTagId(),
                dimensions = tag.GetBounds().size,
                cameraRelativePose = GetObjectPoseInCamera(tag.transform),
                frame = tag.GetComponent<TransformFrame>()
            };
            tagList.Add(tagMsg);
        }
        IncrementMessageCount();
        return tagList.ToArray();
    }
}
