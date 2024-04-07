using System;
using UnityEngine;

[Serializable]
class CapturePassConfig
{
    public string name;
    public string image_topic;
    public string info_topic;
    public RenderTexture renderTexture;
}