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

    // pass configurations
    private CapturePass[] capturePasses = new CapturePass[] {
        new CapturePass() { name = "image", image_topic = "rgb/image_rect_color", info_topic = "rgb/camera_info", publish = true  },
        // new CapturePass() { name = "id", publish = true, supportsAntialiasing = false, mode = ReplacelementModes.ObjectId },
        new CapturePass() { name = "layer", image_topic = "layer/image_raw", info_topic = "layer/camera_info", publish = true, supportsAntialiasing = false, mode = ReplacelementModes.CatergoryId },
        new CapturePass() { name = "depth", image_topic = "depth/depth_registered", info_topic = "depth/camera_info", publish = true, mode = ReplacelementModes.DepthMultichannel, encoding = Encodings.MONO16 },
        // new CapturePass() { name = "normals", publish = true, mode = ReplacelementModes.Normals },
        // new CapturePass() { name = "flow", publish = true, supportsAntialiasing = false, needsRescale = true, mode = ReplacelementModes.Flow }, // (see issue with Motion Vectors in @KNOWN ISSUES)
    };

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
    enum Encodings
    {
        RGB8 = 0,
        MONO16 = 1,
    }

    struct CapturePass
    {
        // configuration
        public string name;
        public string image_topic;
        public string info_topic;
        public bool supportsAntialiasing;
        public bool needsRescale;
        public ReplacelementModes mode;
        public bool publish;
        public Encodings encoding;
        public CapturePass(string name_)
        {
            name = name_;
            supportsAntialiasing = true;
            needsRescale = false;
            camera = null;
            mode = ReplacelementModes.None;
            publish = false;
            encoding = Encodings.RGB8;
            image_topic = "camera/" + name + "/image_raw";
            info_topic = "camera/" + name + "/camera_info";
        }

        // impl
        public Camera camera;
    };

    private ROSConnection ros;
    [SerializeField] private string baseTopic = "camera";
    [SerializeField] private uint imageWidth = 1920;
    [SerializeField] private uint imageHeight = 1080;
    [SerializeField] private float publishRate = 10.0f;
    private float publishStartDelay = 1.0f;
    private CameraInfoMsg cameraInfoMsg;
    private uint seq = 0;
    private TransformFrame frame;

    [SerializeField] private string segmentationTopic = "simulated_segmentation";
    private SegmentationInstanceArrayMsg segmentationMsg = new SegmentationInstanceArrayMsg();

    [SerializeField] private Shader uberReplacementShader;
    [SerializeField] private Shader opticalFlowShader;

    [SerializeField] private float opticalFlowSensitivity = 1.0f;

    // cached materials
    private Material opticalFlowMaterial;

    private Renderer[] prevRenderers = new Renderer[0];

    void Start()
    {
        Camera mainCamera = GetComponent<Camera>();
        frame = GetComponent<TransformFrame>();
        ros = ROSConnection.GetOrCreateInstance();
        foreach (var pass in capturePasses)
        {
            if (pass.publish)
            {
                ros.RegisterPublisher<ImageMsg>(GetImageTopic(pass.image_topic));
                ros.RegisterPublisher<CameraInfoMsg>(GetInfoTopic(pass.info_topic));
            }
        }
        ros.RegisterPublisher<SegmentationInstanceArrayMsg>(baseTopic + "/" + segmentationTopic);

        // default fallbacks, if shaders are unspecified
        if (!uberReplacementShader)
            uberReplacementShader = Shader.Find("Hidden/UberReplacement");

        if (!opticalFlowShader)
            opticalFlowShader = Shader.Find("Hidden/OpticalFlow");

        // use real camera to capture final image
        capturePasses[0].camera = mainCamera;
        for (int q = 1; q < capturePasses.Length; q++)
            capturePasses[q].camera = CreateHiddenCamera(capturePasses[q].name);

        OnCameraChange();
        Renderer[] renderers = FindObjectsOfType<Renderer>();
        OnSceneChange(renderers);

        if (publishRate > 0) {
            InvokeRepeating("PublishTimerCallback", publishStartDelay, 1.0f / publishRate);
        }
    }

    private (uint, uint) FixedAspectResize(uint cameraWidth, uint cameraHeight, uint destinationWidth, uint destinationHeight)
    {
        uint width = destinationWidth;
        uint height = destinationHeight;
        if (cameraWidth * destinationHeight > cameraHeight * destinationWidth)
        {
            height = destinationWidth * cameraHeight / cameraWidth;
        }
        else
        {
            width = destinationHeight * cameraWidth / cameraHeight;
        }
        return (width, height);
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
        return baseTopic + "/" + name + "/image_raw";
    }

    private string GetInfoTopic(string name)
    {
        return baseTopic + "/" + name + "/camera_info";
    }

    void LateUpdate()
    {
        Renderer[] renderers = FindObjectsOfType<Renderer>();

        // OnCameraChange();   // don't change camera parameters at run time
        if (DidSceneChange(renderers))
        {
            OnSceneChange(renderers);
        }
        if (publishRate <= 0)
        {
            PublishTimerCallback();
        }
    }

    private void PublishTimerCallback()
    {
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
        cb.SetGlobalFloat("_OutputMode", (int)mode); // @TODO: CommandBuffer is missing SetGlobalInt() method
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
        Camera mainCamera = GetComponent<Camera>();
        int targetDisplay = 1;
        foreach (var pass in capturePasses)
        {
            if (pass.camera == mainCamera || pass.camera == null)
                continue;

            // cleanup capturing camera
            pass.camera.RemoveAllCommandBuffers();

            // copy all "main" camera parameters into capturing camera
            pass.camera.CopyFrom(mainCamera);

            // set targetDisplay here since it gets overriden by CopyFrom()
            pass.camera.targetDisplay = targetDisplay++;

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
            segmentationMsg = new SegmentationInstanceArrayMsg {
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
        ros.Publish(baseTopic + "/" + segmentationTopic, segmentationMsg);
    }

    private void PublishRenders()
    {
        Camera mainCamera = GetComponent<Camera>();

        cameraInfoMsg = CameraInfoGenerator.ConstructCameraInfoMessage(mainCamera, new HeaderMsg { frame_id = frame.GetFrameId() });
        (uint resizeWidth, uint resizeHeight) = FixedAspectResize(cameraInfoMsg.width, cameraInfoMsg.height, imageWidth, imageHeight);
        resizeCameraInfo(cameraInfoMsg, resizeWidth, resizeHeight);

        HeaderMsg header = new HeaderMsg
        {
            frame_id = frame.GetFrameId(),
            stamp = RosUtil.GetTimeMsg(),
            seq = seq,
        };
        seq++;
        foreach (var pass in capturePasses)
        {
            if (pass.publish)
            {
                PublishImage(pass, header, GetImageTopic(pass.name), (int)resizeWidth, (int)resizeHeight);
                cameraInfoMsg.header = header;
                ros.Publish(GetInfoTopic(pass.name), cameraInfoMsg);
            }
        }
    }

    private void PublishImage(CapturePass pass, HeaderMsg header, string topic, int width, int height)
    {
        Camera mainCamera = GetComponent<Camera>();
        Camera cam = pass.camera;
        var readWrite = RenderTextureReadWrite.Default;
        TextureFormat textureFormat;
        int depth;
        int rosImageChannels;
        string encodingString;
        RenderTextureFormat format;
        switch (pass.encoding)
        {
            case Encodings.RGB8:
                textureFormat = TextureFormat.RGB24;
                rosImageChannels = 3;
                encodingString = "rgb8";
                format = RenderTextureFormat.Default;
                depth = 24;
                break;
            case Encodings.MONO16:
                textureFormat = TextureFormat.R16;
                rosImageChannels = 2;
                encodingString = "16UC1";
                format = RenderTextureFormat.R16;
                depth = 16;
                break;
            default:
                textureFormat = TextureFormat.RGB24;
                rosImageChannels = 0;
                encodingString = "unknown";
                format = RenderTextureFormat.Default;
                depth = 24;
                break;
        }
        var antiAliasing = pass.supportsAntialiasing ? Mathf.Max(1, QualitySettings.antiAliasing) : 1;

        var finalRT =
            RenderTexture.GetTemporary(width, height, depth, format, readWrite, antiAliasing);
        var renderRT = (!pass.needsRescale) ? finalRT :
            RenderTexture.GetTemporary(mainCamera.pixelWidth, mainCamera.pixelHeight, depth, format, readWrite, antiAliasing);
        var texture = new Texture2D(width, height, textureFormat, false);

        var prevActiveRT = RenderTexture.active;
        var prevCameraRT = cam.targetTexture;

        // render to offscreen texture (readonly from CPU side)
        RenderTexture.active = renderRT;
        cam.targetTexture = renderRT;

        cam.Render();

        if (pass.needsRescale)
        {
            // blit to rescale (see issue with Motion Vectors in @KNOWN ISSUES)
            RenderTexture.active = finalRT;
            Graphics.Blit(renderRT, finalRT);
            RenderTexture.ReleaseTemporary(renderRT);
        }

        // flip vertically
        var temp = RenderTexture.GetTemporary(renderRT.descriptor);
        Graphics.Blit(renderRT, temp, new Vector2(1, -1), new Vector2(0, 1));
        Graphics.Blit(temp, renderRT);
        RenderTexture.ReleaseTemporary(temp);

        // read offsreen texture contents into the CPU readable texture
        texture.ReadPixels(new Rect(0, 0, texture.width, texture.height), 0, 0);
        texture.Apply();


        // extract bytes
        byte[] bytes = texture.GetRawTextureData();


        switch (pass.encoding)
        {
            case Encodings.RGB8:
                break;
            case Encodings.MONO16:
                for (int i = 0; i < bytes.Length; i += 2)
                {
                    (bytes[i], bytes[i + 1]) = convertBytesToMillimeters(bytes[i], bytes[i + 1]);
                }
                break;
            default:
                break;
        }

        // publish image
        ImageMsg imageMsg = new ImageMsg(
            header,
            (uint)texture.height,
            (uint)texture.width,
            encodingString,
            0x00,
            (uint)(texture.width * rosImageChannels),
            bytes
        );
        ros.Publish(topic, imageMsg);

        // restore state and cleanup
        cam.targetTexture = prevCameraRT;
        RenderTexture.active = prevActiveRT;

        Destroy(texture);
        RenderTexture.ReleaseTemporary(finalRT);
    }

    (byte, byte) convertBytesToMillimeters(byte lowerByte, byte higherByte)
    {
        uint unscaledRawDepth = (uint)(lowerByte | (higherByte << 8));
        if (unscaledRawDepth == 0)
        {
            return (0, 0);
        }
        float depthMeters = 1.0f / 0xffff * unscaledRawDepth;
        depthMeters = 3.90625f * (depthMeters - 0.256f) + 1.0f;
        uint depthMillimeters = (uint)(1000.0f * depthMeters);
        byte rawDepthLowerByte = (byte)(depthMillimeters & 0xff);
        byte rawDepthHigherByte = (byte)((depthMillimeters >> 8) & 0xff);
        return (rawDepthLowerByte, rawDepthHigherByte);
    }
}
