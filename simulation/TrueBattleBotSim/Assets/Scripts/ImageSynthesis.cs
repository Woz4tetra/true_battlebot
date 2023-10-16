using UnityEngine;
using UnityEngine.Rendering;
using System.Collections;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

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
        new CapturePass() { name = "image" },
        // new CapturePass() { name = "id", supportsAntialiasing = false, mode = ReplacelementModes.ObjectId, publish = true },
        new CapturePass() { name = "layer", supportsAntialiasing = false, mode = ReplacelementModes.CatergoryId, publish = true },
        new CapturePass() { name = "depth", mode = ReplacelementModes.DepthMultichannel, publish = true, encoding = Encodings.MONO16 },
        // new CapturePass() { name = "normals", mode = ReplacelementModes.Normals, publish = true },
        // new CapturePass() { name = "flow", supportsAntialiasing = false, needsRescale = true, publish = true, mode = ReplacelementModes.Flow }, // (see issue with Motion Vectors in @KNOWN ISSUES)
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
        }

        // impl
        public Camera camera;
    };

    private ROSConnection ros;
    [SerializeField] private string baseTopic = "simulated_camera";
    [SerializeField] private string frameId = "camera";
    [SerializeField] private uint imageWidth = 1920;
    [SerializeField] private uint imageHeight = 1080;
    [SerializeField] private float publishRate = 10.0f;
    [SerializeField] private float minDepth = 0.5f;
    [SerializeField] private float maxDepth = 4.4062f;
    private float publishStartDelay = 1.0f;
    private CameraInfoMsg cameraInfoMsg;
    private uint seq = 0;

    [SerializeField] private Shader uberReplacementShader;
    [SerializeField] private Shader opticalFlowShader;

    [SerializeField] private float opticalFlowSensitivity = 1.0f;

    // cached materials
    private Material opticalFlowMaterial;

    void Start()
    {
        Camera mainCamera = GetComponent<Camera>();
        ros = ROSConnection.GetOrCreateInstance();
        foreach (var pass in capturePasses)
        {
            if (pass.publish)
            {
                ros.RegisterPublisher<ImageMsg>(GetImageTopic(pass.name));
                ros.RegisterPublisher<CameraInfoMsg>(GetInfoTopic(pass.name));
            }
        }

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
        OnSceneChange();

        InvokeRepeating("PublishTimerCallback", publishStartDelay, 1.0f / publishRate);
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
#if UNITY_EDITOR
        if (DetectPotentialSceneChangeInEditor())
            OnSceneChange();
#endif // UNITY_EDITOR

        // @TODO: detect if camera properties actually changed
        OnCameraChange();
    }

    private void PublishTimerCallback()
    {
        Publish();
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


    public void OnSceneChange(bool grayscale = false)
    {
        var renderers = Object.FindObjectsOfType<Renderer>();
        var mpb = new MaterialPropertyBlock();
        foreach (var r in renderers)
        {
            var id = r.gameObject.GetInstanceID();
            var layer = r.gameObject.layer;

            mpb.SetColor("_ObjectColor", ColorEncoding.EncodeIDAsColor(id));
            mpb.SetColor("_CategoryColor", ColorEncoding.EncodeLayerAsColor(layer, grayscale));
            r.SetPropertyBlock(mpb);
        }
    }

    private void Publish()
    {
        Camera mainCamera = GetComponent<Camera>();

        cameraInfoMsg = CameraInfoGenerator.ConstructCameraInfoMessage(mainCamera, new HeaderMsg { frame_id = frameId });
        (uint resizeWidth, uint resizeHeight) = FixedAspectResize(cameraInfoMsg.width, cameraInfoMsg.height, imageWidth, imageHeight);
        resizeCameraInfo(cameraInfoMsg, resizeWidth, resizeHeight);

        foreach (var pass in capturePasses)
        {
            if (pass.publish)
            {
                HeaderMsg header = new HeaderMsg
                {
                    frame_id = frameId,
                    stamp = RosUtil.GetTimeMsg(),
                    seq = seq,
                };
                seq++;
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
        float depthMeters = (maxDepth - minDepth) / 0xffff * unscaledRawDepth + minDepth;
        uint depthMillimeters = (uint)(1000.0f * depthMeters);
        byte rawDepthLowerByte = (byte)(depthMillimeters & 0xff);
        byte rawDepthHigherByte = (byte)((depthMillimeters >> 8) & 0xff);
        return (rawDepthLowerByte, rawDepthHigherByte);
    }

#if UNITY_EDITOR
    private GameObject lastSelectedGO;
    private int lastSelectedGOLayer = -1;
    private string lastSelectedGOTag = "unknown";
    private bool DetectPotentialSceneChangeInEditor()
    {
        bool change = false;
        // there is no callback in Unity Editor to automatically detect changes in scene objects
        // as a workaround lets track selected objects and check, if properties that are 
        // interesting for us (layer or tag) did not change since the last frame
        if (UnityEditor.Selection.transforms.Length > 1)
        {
            // multiple objects are selected, all bets are off!
            // we have to assume these objects are being edited
            change = true;
            lastSelectedGO = null;
        }
        else if (UnityEditor.Selection.activeGameObject)
        {
            var go = UnityEditor.Selection.activeGameObject;
            // check if layer or tag of a selected object have changed since the last frame
            var potentialChangeHappened = lastSelectedGOLayer != go.layer || lastSelectedGOTag != go.tag;
            if (go == lastSelectedGO && potentialChangeHappened)
                change = true;

            lastSelectedGO = go;
            lastSelectedGOLayer = go.layer;
            lastSelectedGOTag = go.tag;
        }

        return change;
    }
#endif // UNITY_EDITOR
}
