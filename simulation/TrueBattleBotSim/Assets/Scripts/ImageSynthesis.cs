using UnityEngine;
using UnityEngine.Rendering;
using System.Collections;
using System.IO;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.BuiltinInterfaces;

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
        new CapturePass() { name = "depth", mode = ReplacelementModes.DepthMultichannel, publish = true, encoding = Encodings.F16 },
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
        F16 = 1,
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
        cameraInfoMsg = CameraInfoGenerator.ConstructCameraInfoMessage(mainCamera, new HeaderMsg { frame_id = frameId });

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
        var go = new GameObject(name, typeof(Camera));
        go.hideFlags = HideFlags.HideAndDontSave;
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
        int targetDisplay = 1;
        var mainCamera = GetComponent<Camera>();
        foreach (var pass in capturePasses)
        {
            if (pass.camera == mainCamera)
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
            var tag = r.gameObject.tag;

            mpb.SetColor("_ObjectColor", ColorEncoding.EncodeIDAsColor(id));
            mpb.SetColor("_CategoryColor", ColorEncoding.EncodeLayerAsColor(layer, grayscale));
            r.SetPropertyBlock(mpb);
        }
    }

    public void Publish()
    {
        // execute as coroutine to wait for the EndOfFrame before starting capture
        StartCoroutine(WaitForEndOfFrameAndPublish(imageWidth, imageHeight));
    }

    private IEnumerator WaitForEndOfFrameAndPublish(uint width, uint height)
    {
        yield return new WaitForEndOfFrame();
        Publish(width, height);
    }

    private void Publish(uint width, uint height)
    {
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
                Publish(pass, header, GetImageTopic(pass.name), (int)width, (int)height);
                cameraInfoMsg.header = header;
                ros.Publish(GetInfoTopic(pass.name), cameraInfoMsg);
            }
        }
    }

    private void Publish(CapturePass pass, HeaderMsg header, string topic, int width, int height)
    {
        Camera cam = pass.camera;
        var mainCamera = GetComponent<Camera>();
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
            case Encodings.F16:
                textureFormat = TextureFormat.R16;
                rosImageChannels = 4;
                encodingString = "32SC1";
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
        byte[] bytes;

        switch (pass.encoding)
        {
            case Encodings.RGB8:
                bytes = texture.GetRawTextureData();
                break;
            case Encodings.F16:
                Color[] pixels = texture.GetPixels();
                float[] pixelFloats = new float[pixels.Length];
                for (int i = 0; i < pixels.Length; i++)
                {
                    pixelFloats[i] = pixels[i].r;
                }

                bytes = new byte[pixelFloats.Length * sizeof(float)];
                System.Buffer.BlockCopy(pixelFloats, 0, bytes, 0, bytes.Length);
                break;
            default:
                bytes = new byte[0];
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
