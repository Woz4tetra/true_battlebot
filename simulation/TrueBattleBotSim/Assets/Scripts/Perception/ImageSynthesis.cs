using UnityEngine;
using UnityEngine.Rendering;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.BwInterfaces;
using System.Collections.Generic;

// @TODO:
// . support custom color wheels in optical flow via lookup textures
// . support custom depth encoding
// . support multiple overlay cameras
// . tests
// . better example scene(s)

// @KNOWN ISSUES
// . Motion Vectors can produce incorrect results in Unity 5.5.f3 when
//      1) during the first rendering frame
//      2) rendering several cameras with different aspect ratios - vectors do stretch to the sides of the screen

[RequireComponent(typeof(Camera))]
public class ImageSynthesis : MonoBehaviour
{
    [SerializeField] private string baseTopic = "camera";
    [SerializeField] private uint imageWidth = 1920;
    [SerializeField] private uint imageHeight = 1080;
    [SerializeField] private float publishRate = 10.0f;
    [SerializeField] private string segmentationTopic = "simulated_segmentation";
    [SerializeField] private bool publishSegmentationLabels = true;
    [SerializeField] CapturePassConfig[] captureConfig;
    [SerializeField] private Shader uberReplacementShader;
    [SerializeField] private Shader opticalFlowShader;

    [SerializeField] private float opticalFlowSensitivity = 1.0f;

    private CapturePass[] capturePasses;

    enum ReplacelementModes
    {
        None = -1,
        ObjectId = 0,
        CatergoryId = 1,
        DepthCompressed = 2,
        DepthMultichannel = 3,
        Normals = 4,
        Flow = 5,
    };
    enum Encoding
    {
        RGB8 = 0,
        MONO16 = 1,
    }

    class CapturePass
    {
        // configuration
        public string name;
        public string imageTopic;
        public string infoTopic;
        public string requestTopic;
        public bool supportsAntialiasing = true;
        public bool needsRescale = false;
        public ReplacelementModes mode = ReplacelementModes.None;
        public Encoding encoding = Encoding.RGB8;
        public int targetDisplay = 1;

        public uint outputWidth;
        public uint outputHeight;

        public RenderTexture renderTexture;
        public Camera camera;
        public RosTopicState imageTopicState;
        public RosTopicState infoTopicState;
        public RosPollSubscriber<EmptyMsg> requestTopicSub;
    };

    private ROSConnection ros;

    private CameraInfoMsg cameraInfoMsg;
    private uint seq = 0;
    private TransformFrame frame;
    private float prevPublishTime = 0.0f;

    private SegmentationInstanceArrayMsg segmentationMsg = new SegmentationInstanceArrayMsg();


    // cached materials
    private Material opticalFlowMaterial;

    private Renderer[] prevRenderers = new Renderer[0];

    private Camera mainCamera;
    bool skipFrameRender = false;

    void Start()
    {
        mainCamera = GetComponent<Camera>();
        frame = ObjectUtils.GetComponentInTree<TransformFrame>(gameObject);
        Renderer[] renderers = FindObjectsOfType<Renderer>();

        ros = ROSConnection.GetOrCreateInstance();
        List<CapturePass> passes = new List<CapturePass>();

        skipFrameRender = isHeadless();
        if (skipFrameRender)
        {
            Debug.LogWarning("Running in headless mode, disabling image synthesis");
        }

        foreach (CapturePassConfig config in captureConfig)
        {
            if (!config.enabled)
            {
                continue;
            }
            CapturePass pass = new CapturePass()
            {
                name = config.name,
                imageTopic = config.imageTopic,
                infoTopic = config.infoTopic,
                requestTopic = config.continuous ? "" : config.requestTopic,
                renderTexture = config.renderTexture
            };
            bool is_set = true;
            switch (config.name)
            {
                case "image":
                    break;
                case "layer":
                    pass.supportsAntialiasing = false;
                    pass.mode = ReplacelementModes.CatergoryId;
                    break;
                case "depth":
                    pass.mode = ReplacelementModes.DepthMultichannel;
                    pass.encoding = Encoding.MONO16;
                    break;
                case "id":
                    pass.supportsAntialiasing = false;
                    pass.mode = ReplacelementModes.ObjectId;
                    break;
                case "normals":
                    pass.mode = ReplacelementModes.Normals;
                    break;
                case "flow":
                    pass.supportsAntialiasing = false;
                    pass.needsRescale = true;
                    pass.mode = ReplacelementModes.Flow;
                    break;
                default:
                    Debug.LogError("Unknown capture pass key: " + config.name);
                    is_set = false;
                    break;
            }
#if UNITY_EDITOR
            pass.needsRescale = true;  // force rescale for camera in editor mode
#endif
            if (is_set)
            {
                passes.Add(pass);
            }
        }
        foreach (CapturePass pass in passes)
        {
            pass.imageTopicState = ros.GetTopic(GetImageTopic(pass.imageTopic));
            pass.infoTopicState = ros.GetTopic(GetImageTopic(pass.infoTopic));
            if (pass.imageTopicState == null)
            {
                pass.imageTopicState = ros.RegisterPublisher<ImageMsg>(GetImageTopic(pass.imageTopic), queue_size: 10);
            }
            if (pass.infoTopicState == null)
            {
                pass.infoTopicState = ros.RegisterPublisher<CameraInfoMsg>(GetImageTopic(pass.infoTopic), queue_size: 10);
            }
            if (pass.requestTopic.Length > 0)
            {
                pass.requestTopicSub = new RosPollSubscriber<EmptyMsg>(GetImageTopic(pass.requestTopic));
            }
        }
        if (publishSegmentationLabels)
        {
            RosTopicState segmentationTopicState = ros.GetTopic(GetImageTopic(segmentationTopic));
            if (segmentationTopicState == null)
            {
                ros.RegisterPublisher<SegmentationInstanceArrayMsg>(GetImageTopic(segmentationTopic), queue_size: 10);
            }
        }

        // default fallbacks, if shaders are unspecified
        if (!uberReplacementShader)
            uberReplacementShader = Shader.Find("Hidden/UberReplacement");

        if (!opticalFlowShader)
            opticalFlowShader = Shader.Find("Hidden/OpticalFlow");

        capturePasses = passes.ToArray();
        // use real camera to capture final image
        for (int q = 0; q < capturePasses.Length; q++)
        {
            string name = capturePasses[q].name;
            capturePasses[q].camera = name == "image" ? mainCamera : CreateHiddenCamera(capturePasses[q].name);
        }

        OnCameraChange();
        OnSceneChange(renderers);
    }

    private void resizeCameraInfo(CameraInfoMsg cameraInfoMsg, uint destinationWidth, uint destinationHeight)
    {
        float scale_y = (float)destinationHeight / cameraInfoMsg.height;
        float scale_x = (float)destinationWidth / cameraInfoMsg.width;
        cameraInfoMsg.height = destinationHeight;
        cameraInfoMsg.width = destinationWidth;

        cameraInfoMsg.K[0] *= scale_x;  // fx
        cameraInfoMsg.K[2] *= scale_x;  // cx
        cameraInfoMsg.K[4] *= scale_y;  // fy
        cameraInfoMsg.K[5] *= scale_y;  // cy

        cameraInfoMsg.P[0] *= scale_x;  // fx
        cameraInfoMsg.P[2] *= scale_x;  // cx
        cameraInfoMsg.P[3] *= scale_x;  // T
        cameraInfoMsg.P[5] *= scale_y;  // fy
        cameraInfoMsg.P[6] *= scale_y;  // cy

        cameraInfoMsg.roi.x_offset = (uint)(cameraInfoMsg.roi.x_offset * scale_x);
        cameraInfoMsg.roi.y_offset = (uint)(cameraInfoMsg.roi.y_offset * scale_y);
        cameraInfoMsg.roi.width = (uint)(cameraInfoMsg.roi.width * scale_x);
        cameraInfoMsg.roi.height = (uint)(cameraInfoMsg.roi.height * scale_y);
    }

    private string GetImageTopic(string name)
    {
        return baseTopic + "/" + name;
    }

    void LateUpdate()
    {
        Renderer[] renderers = FindObjectsOfType<Renderer>();

        // OnCameraChange();   // don't change camera parameters at run time
        if (DidSceneChange(renderers))
        {
            OnSceneChange(renderers);
        }
        if (publishRate <= 0 || Time.unscaledTime - prevPublishTime > 1.0f / publishRate)
        {
            PublishTimerCallback();
            prevPublishTime = Time.unscaledTime;
        }
    }

    void PublishTimerCallback()
    {
        if (!gameObject.activeInHierarchy)
        {
            return;
        }
        if (!ros.HasConnectionThread)
        {
            return;
        }
        PublishRenders();
        PublishLabels();
    }

    private Camera CreateHiddenCamera(string name)
    {
        var go = new GameObject(name, typeof(Camera))
        {
            hideFlags = HideFlags.HideAndDontSave
        };
        go.transform.parent = transform;

        var newCamera = go.GetComponent<Camera>();
        return newCamera;
    }


    static private void SetupCameraWithReplacementShader(Camera cam, Shader shader, ReplacelementModes mode)
    {
        SetupCameraWithReplacementShader(cam, shader, mode, Color.black);
    }

    static private void SetupCameraWithReplacementShader(Camera cam, Shader shader, ReplacelementModes mode, Color clearColor)
    {
        var cb = new CommandBuffer();
        cb.SetGlobalFloat("_OutputMode", (int)mode);
        cb.SetGlobalFloat("_DepthRescale", cam.farClipPlane / 1000.0f);  // scale based on reference clip plane (1000)
        cam.AddCommandBuffer(CameraEvent.BeforeForwardOpaque, cb);
        cam.AddCommandBuffer(CameraEvent.BeforeFinalPass, cb);
        cam.SetReplacementShader(shader, "");
        cam.backgroundColor = clearColor;
        cam.clearFlags = CameraClearFlags.SolidColor;
    }

    static private void SetupCameraWithPostShader(Camera cam, Material material, DepthTextureMode depthTextureMode = DepthTextureMode.None)
    {
        var cb = new CommandBuffer();
        cb.Blit(null, BuiltinRenderTextureType.CurrentActive, material);
        cam.AddCommandBuffer(CameraEvent.AfterEverything, cb);
        cam.depthTextureMode = depthTextureMode;
    }


    public void OnCameraChange()
    {
        int targetDisplay = mainCamera.targetDisplay;
        foreach (CapturePass pass in capturePasses)
        {
            if (pass.camera == mainCamera || pass.camera == null)
                continue;

            // cleanup capturing camera
            pass.camera.RemoveAllCommandBuffers();

            // copy all "main" camera parameters into capturing camera
            pass.camera.CopyFrom(mainCamera);

            // set targetDisplay here since it gets overriden by CopyFrom()
            pass.camera.targetDisplay = ++targetDisplay;
            Debug.Log($"Setting target display to {pass.camera.targetDisplay} for camera {pass.camera.name}");

            // setup command buffers and replacement shaders
            if (pass.mode == ReplacelementModes.Flow)
            {
                SetupCameraWithPostShader(pass.camera, opticalFlowMaterial, DepthTextureMode.Depth | DepthTextureMode.MotionVectors);
            }
            else
            {
                SetupCameraWithReplacementShader(pass.camera, uberReplacementShader, pass.mode);
            }
        }

        // cache materials and setup material properties
        if (!opticalFlowMaterial || opticalFlowMaterial.shader != opticalFlowShader)
            opticalFlowMaterial = new Material(opticalFlowShader);
        opticalFlowMaterial.SetFloat("_Sensitivity", opticalFlowSensitivity);
    }

    bool DidSceneChange(Renderer[] renderers)
    {
        return renderers.Length != prevRenderers.Length;
    }

    public void OnSceneChange(Renderer[] renderers, bool grayscale = false)
    {
        var mpb = new MaterialPropertyBlock();
        List<SegmentationInstanceMsg> segmentation = new List<SegmentationInstanceMsg>();
        foreach (Renderer render in renderers)
        {
            var id = render.gameObject.GetInstanceID();
            var layer = render.gameObject.layer;
            string label = LayerMask.LayerToName(layer);

            Color objectColor = ColorEncoding.EncodeIDAsColor(id);
            Color layerColor = ColorEncoding.EncodeLayerAsColor(layer, grayscale);

            uint classId = GetLayerCode(layerColor);

            mpb.SetColor("_ObjectColor", objectColor);
            mpb.SetColor("_CategoryColor", layerColor);
            render.SetPropertyBlock(mpb);

            segmentation.Add(new SegmentationInstanceMsg
            {
                score = 1.0f,
                label = label,
                class_index = classId,
                object_index = (uint)id,
                has_holes = false,
            });
        }
        if (cameraInfoMsg != null)
        {
            segmentationMsg = new SegmentationInstanceArrayMsg
            {
                header = cameraInfoMsg.header,
                height = cameraInfoMsg.height,
                width = cameraInfoMsg.width,
                instances = segmentation.ToArray(),
            };
        }
    }

    private uint GetLayerCode(Color layerColor)
    {
        uint layerCode = 0;
        layerCode |= (uint)(layerColor.r * 255) << 16;
        layerCode |= (uint)(layerColor.g * 255) << 8;
        layerCode |= (uint)(layerColor.b * 255);
        return layerCode;
    }

    private void PublishLabels()
    {
        if (publishSegmentationLabels)
        {
            ros.Publish(baseTopic + "/" + segmentationTopic, segmentationMsg);
        }
    }

    private void PublishRenders()
    {
        cameraInfoMsg = CameraInfoGenerator.ConstructCameraInfoMessage(mainCamera, new HeaderMsg { frame_id = frame.GetFrameId() });
        resizeCameraInfo(cameraInfoMsg, imageWidth, imageHeight);

        cameraInfoMsg.header = new HeaderMsg
        {
            frame_id = frame.GetFrameId(),
            stamp = RosUtil.GetTimeMsg(),
            seq = seq,
        };
        seq++;

        if (skipFrameRender)
        {
            foreach (CapturePass pass in capturePasses)
            {
                PublishInfo(pass, cameraInfoMsg);
            }
            return;
        }

        Dictionary<string, Texture2D> imagesToPublish = new Dictionary<string, Texture2D>();

        foreach (CapturePass pass in capturePasses)
        {
            if (pass.requestTopic.Length > 0)
            {
                if (pass.requestTopicSub.Receive().TryGet(out EmptyMsg request))
                {
                    Debug.Log("Received request for " + pass.name);
                }
                else
                {
                    continue;
                }
            }

            pass.outputWidth = imageWidth;
            pass.outputHeight = imageHeight;
            Texture2D texture = RenderPassToTexture(pass);
            imagesToPublish[pass.name] = texture;
        }
        foreach (CapturePass pass in capturePasses)
        {
            if (imagesToPublish.ContainsKey(pass.name))
            {
                ImageMsg imageMsg = textureToImageMsg(pass, imagesToPublish[pass.name]);
                PublishImage(pass, imageMsg, cameraInfoMsg.header);
                PublishInfo(pass, cameraInfoMsg);
            }
        }
    }

    private ImageMsg textureToImageMsg(CapturePass pass, Texture2D texture)
    {
        Encoding encoding = pass.encoding;
        int rosImageChannels;
        string encodingString;
        switch (encoding)
        {
            case Encoding.RGB8:
                rosImageChannels = 3;
                encodingString = "rgb8";
                break;
            case Encoding.MONO16:
                rosImageChannels = 2;
                encodingString = "16UC1";
                break;
            default:
                rosImageChannels = 0;
                encodingString = "unknown";
                break;
        }
        // extract bytes
        byte[] bytes = texture.GetRawTextureData();

        Destroy(texture);

        // create ROS Image message
        return new ImageMsg(
            new HeaderMsg(),
            (uint)texture.height,
            (uint)texture.width,
            encodingString,
            0x00,
            (uint)(texture.width * rosImageChannels),
            bytes
        );

    }

    private Texture2D RenderPassToTexture(CapturePass pass)
    {
        Camera camera = pass.camera;
        int width = (int)pass.outputWidth;
        int height = (int)pass.outputHeight;
        int cameraWidth = mainCamera.pixelWidth;
        int cameraHeight = mainCamera.pixelHeight;
        Encoding encoding = pass.encoding;
        RenderTexture renderTexture = pass.renderTexture;
        bool supportsAntialiasing = pass.supportsAntialiasing;
        bool needsRescale = pass.needsRescale;

        var readWrite = RenderTextureReadWrite.Default;
        TextureFormat textureFormat;
        int depth;

        RenderTextureFormat format;
        switch (encoding)
        {
            case Encoding.RGB8:
                textureFormat = TextureFormat.RGB24;
                format = RenderTextureFormat.Default;
                depth = 24;
                break;
            case Encoding.MONO16:
                textureFormat = TextureFormat.R16;
                format = RenderTextureFormat.R16;
                depth = 16;
                break;
            default:
                textureFormat = TextureFormat.RGB24;
                format = RenderTextureFormat.Default;
                depth = 24;
                break;
        }
        var antiAliasing = supportsAntialiasing ? Mathf.Max(1, QualitySettings.antiAliasing) : 1;

        var finalRT =
            RenderTexture.GetTemporary(width, height, depth, format, readWrite, antiAliasing);
        var renderRT = (!needsRescale) ? finalRT :
            RenderTexture.GetTemporary(cameraWidth, cameraHeight, depth, format, readWrite, antiAliasing);
        var texture = new Texture2D(width, height, textureFormat, false);

        var prevActiveRT = RenderTexture.active;
        var prevCameraRT = camera.targetTexture;

        // render to offscreen texture (readonly from CPU side)
        RenderTexture.active = renderRT;
        camera.targetTexture = renderRT;

        camera.Render();

        if (renderTexture != null)
        {
            Graphics.Blit(renderRT, renderTexture);
        }

        // flip vertically
        var temp = RenderTexture.GetTemporary(renderRT.descriptor);
        Graphics.Blit(renderRT, temp, new Vector2(1, -1), new Vector2(0, 1));
        Graphics.Blit(temp, renderRT);
        RenderTexture.ReleaseTemporary(temp);

        if (needsRescale)
        {
            // blit to rescale (see issue with Motion Vectors in @KNOWN ISSUES)
            RenderTexture.active = finalRT;
            Graphics.Blit(renderRT, finalRT);
            RenderTexture.ReleaseTemporary(renderRT);
        }

        // read offsreen texture contents into the CPU readable texture
        texture.ReadPixels(new Rect(0, 0, texture.width, texture.height), 0, 0);
        texture.Apply();

        // restore state and cleanup
        camera.targetTexture = prevCameraRT;
        RenderTexture.active = prevActiveRT;

        RenderTexture.ReleaseTemporary(finalRT);

        return texture;
    }
    private void PublishImage(CapturePass pass, ImageMsg imageMsg, HeaderMsg header)
    {
        imageMsg.header = header;
        pass.imageTopicState.Publish(imageMsg);
    }

    private void PublishInfo(CapturePass pass, CameraInfoMsg cameraInfoMsg)
    {
        pass.infoTopicState.Publish(cameraInfoMsg);
    }

    private bool isHeadless()
    {
        return SystemInfo.graphicsDeviceType == GraphicsDeviceType.Null;
    }
}
