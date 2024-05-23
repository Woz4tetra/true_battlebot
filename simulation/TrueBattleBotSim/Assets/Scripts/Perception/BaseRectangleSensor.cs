using UnityEngine;
using System.Collections.Generic;
using RosMessageTypes.Std;

public abstract class BaseRectangleSensor : BaseGameObjectSensor
{
    override protected VisibleTarget[] ProcessObjects(GameObject[] objs)
    {
        List<VisibleTarget> tagList = new List<VisibleTarget>();
        foreach (GameObject obj in objs)
        {
            RectangleTarget tag = obj.GetComponent<RectangleTarget>();
            if (tag == null || !IsVisible(obj))
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
                dimensions = tag.GetDimensions(),
                cameraRelativePose = GetObjectPoseInCamera(tag.transform),
                frame = obj.GetComponent<TransformFrame>()
            };
            tagList.Add(tagMsg);
        }
        IncrementMessageCount();
        return tagList.ToArray();
    }
}
