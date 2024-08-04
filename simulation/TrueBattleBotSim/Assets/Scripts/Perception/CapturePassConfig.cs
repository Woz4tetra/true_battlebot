using System;
using UnityEngine;

[Serializable]
class CapturePassConfig
{
    public string name;
    public string imageTopic;
    public string infoTopic;
    public string requestTopic;
    public RenderTexture renderTexture;
    public bool enabled = true;
    public bool continuous = true;
}