using RosMessageTypes.BwInterfaces;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class UiManager : MonoBehaviour
{
    [SerializeField] TMP_Dropdown resolutionDropdown;
    [SerializeField] TMP_Dropdown qualityDropdown;
    [SerializeField] TMP_Dropdown scenarioDropdown;
    [SerializeField] Toggle toggleFullscreen;
    [SerializeField] Toggle enableSimulatedCamerasToggle;
    [SerializeField] TMP_Text fpsReadout;
    [SerializeField] GameObject settingsPanel;
    [SerializeField] GameObject settingsBackground;
    [SerializeField] GameObject enterSettingsPanel;
    [SerializeField] GameObject displayReadout;
    [SerializeField] GameObject simulatedCameras;
    [SerializeField] FPSCounter fpsCounter;
    [SerializeField] CameraController cameraController;
    [SerializeField] FieldManager sceneManager;
    [SerializeField] string remoteScenarioSelectionTopic = "simulation/scenario_selection";
    [SerializeField] string scenarioListTopic = "simulation/scenarios";

    DisplayReadoutManager displayReadoutManager;

    Resolution[] resolutions;
    List<string> scenarios = new List<string>();
    Dictionary<string, int> scenarioIndex = new Dictionary<string, int>();

    bool isShown = false;
    bool wasPausedWhenSettingsOpened = false;
    int commonWidth = 1920;
    int commonHeight = 1080;

    ROSConnection ros;

    public enum TextureQualityPreset
    {
        VERY_LOW = 0,
        LOW = 1,
        MEDIUM = 2,
        HIGH = 3,
        VERY_HIGH = 4,
        ULTRA = 5
    }

    public class PreferenceKey
    {
        private PreferenceKey(string value) { Value = value; }
        public string Value { get; set; }

        public static PreferenceKey QualitySettingPreference { get { return new PreferenceKey("QualitySettingPreference"); } }
        public static PreferenceKey ResolutionPreference { get { return new PreferenceKey("ResolutionPreference"); } }
        public static PreferenceKey FullscreenPreference { get { return new PreferenceKey("FullscreenPreference"); } }
        public static PreferenceKey ScenarioPreference { get { return new PreferenceKey("ScenarioPreference"); } }
        public static PreferenceKey SimulatedCameraPreference { get { return new PreferenceKey("SimulatedCameraPreference"); } }

        public override string ToString()
        {
            return Value;
        }
    }

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<StringMsg>(remoteScenarioSelectionTopic, RemoteScenarioSelectionCallback);
        ros.RegisterPublisher<LabelsMsg>(scenarioListTopic);

        resolutionDropdown.ClearOptions();
        List<string> options = new List<string>();
        resolutions = Screen.resolutions;
        int currentResolutionIndex = -1;

        for (int i = 0; i < resolutions.Length; i++)
        {
            string option = resolutions[i].width + " x " + resolutions[i].height;
            options.Add(option);
            if (currentResolutionIndex != -1)
            {
                continue;
            }
            if (resolutions[i].width == commonWidth && resolutions[i].height == commonHeight)
            {
                currentResolutionIndex = i;
                Debug.Log(
                    $"Common resolution found: ({commonWidth} x {commonHeight}). " +
                    $"Labeling as index {currentResolutionIndex}"
                );
            }
            else if (resolutions[i].width == Screen.currentResolution.width
                    && resolutions[i].height == Screen.currentResolution.height)
            {
                currentResolutionIndex = i;
                Debug.Log(
                    $"Current resolution found: ({commonWidth} x {commonHeight}). " +
                    $"Labeling as index {currentResolutionIndex}"
                );
            }
        }

        scenarios = new List<string>(sceneManager.GetScenarioNames());
        scenarios.Insert(0, "");
        for (int i = 0; i < scenarios.Count; i++)
        {
            scenarioIndex[scenarios[i]] = i;
        }
        scenarioDropdown.ClearOptions();
        scenarioDropdown.AddOptions(scenarios);
        scenarioDropdown.value = 0;
        scenarioDropdown.RefreshShownValue();

        resolutionDropdown.AddOptions(options);
        resolutionDropdown.RefreshShownValue();
        LoadSettings(currentResolutionIndex);
        if (settingsPanel.activeSelf)
        {
            wasPausedWhenSettingsOpened = true;
        }

        displayReadoutManager = displayReadout.GetComponent<DisplayReadoutManager>();

        ShowHideSettingsPanel(settingsPanel.activeSelf);
        StartCoroutine(PublishScenarioListCallback());
        StartCoroutine(UpdateFpsCounter());
    }

    public void SetFullscreenCallback(bool isFullscreen)
    {
        SetFullscreen(isFullscreen);
    }
    public void SetResolutionCallback(int resolutionIndex)
    {
        SetResolution(resolutionIndex);
    }
    public void SetQualityCallback(int qualityIndex)
    {
        SetQuality(qualityIndex);
    }
    public void SetScenarioCallback(int scenarioIndex)
    {
        SetScenario(scenarioIndex);
    }

    public void SetEnableSimulatedCamerasCallback(bool enableCameras)
    {
        SetEnableSimulatedCameras(enableCameras);
    }

    private void SetFullscreen(bool isFullscreen)
    {
        Screen.fullScreen = isFullscreen;
        PlayerPrefs.SetInt(PreferenceKey.FullscreenPreference.Value, Convert.ToInt32(isFullscreen));
        PlayerPrefs.Save();
    }

    private void SetResolution(int resolutionIndex)
    {
        Debug.Log($"Setting resolution to {resolutionIndex}");
        Resolution resolution = resolutions[resolutionIndex];
        Screen.SetResolution(resolution.width, resolution.height, Screen.fullScreen);
        Debug.Log($"Screen resolution is {resolution.width} x {resolution.height}");
        PlayerPrefs.SetInt(PreferenceKey.ResolutionPreference.Value, resolutionDropdown.value);
        PlayerPrefs.Save();
    }

    private void SetScenario(int scenarioIndex)
    {
        string scenarioName = scenarios[scenarioIndex];
        Debug.Log($"Setting scenario to {scenarioIndex} -> {scenarioName}");
        PlayerPrefs.SetString(PreferenceKey.ScenarioPreference.Value, scenarioName);
        PlayerPrefs.Save();
        sceneManager.LoadScenario(scenarioName);
    }

    private void SetEnableSimulatedCameras(bool enableCameras)
    {
        simulatedCameras.SetActive(enableCameras);
        Debug.Log($"Set enable simulated cameras to {enableCameras}");
        PlayerPrefs.SetInt(PreferenceKey.SimulatedCameraPreference.Value, Convert.ToInt32(enableCameras));
        PlayerPrefs.Save();
    }

    private void SetTextureQuality(int textureIndex)
    {
        Debug.Log($"Set texture quality to {textureIndex}");
        QualitySettings.globalTextureMipmapLimit = textureIndex;
    }
    private void SetAntiAliasing(int aaIndex)
    {
        Debug.Log($"Set anti aliasing to {aaIndex}");
        QualitySettings.antiAliasing = aaIndex;
    }

    public void ShowHideSettingsPanel(bool show)
    {
        isShown = show;
        settingsPanel.SetActive(show);
        settingsBackground.SetActive(show);
        enterSettingsPanel.gameObject.SetActive(!show);
        displayReadoutManager.SetEnableClicks(!show);
        cameraController.EnableControls(!show);
        if (show)
        {
            wasPausedWhenSettingsOpened = IsPaused();
            SetPause(true);
        }
        else
        {
            SetPause(wasPausedWhenSettingsOpened);
        }
    }

    public void SetPause(bool paused)
    {
        sceneManager.GetPauseManager().SetPause(paused);
    }

    public bool IsPaused()
    {
        return sceneManager.GetPauseManager().IsPaused();
    }

    private void SetQuality(int qualityIndex)
    {
        Debug.Log($"Set quality to {qualityIndex}");
        QualitySettings.SetQualityLevel(qualityIndex);
        TextureQualityPreset quality = (TextureQualityPreset)qualityIndex;
        int textureValue = 0;
        int antialiasingValue = 0;

        switch (quality)
        {
            case TextureQualityPreset.VERY_LOW:
                textureValue = 3;
                antialiasingValue = 0;
                break;
            case TextureQualityPreset.LOW:
                textureValue = 2;
                antialiasingValue = 0;
                break;
            case TextureQualityPreset.MEDIUM:
                textureValue = 1;
                antialiasingValue = 0;
                break;
            case TextureQualityPreset.HIGH:
                textureValue = 0;
                antialiasingValue = 0;
                break;
            case TextureQualityPreset.VERY_HIGH:
                textureValue = 0;
                antialiasingValue = 1;
                break;
            case TextureQualityPreset.ULTRA:
                textureValue = 0;
                antialiasingValue = 2;
                break;
        }

        qualityDropdown.value = qualityIndex;
        SetTextureQuality(textureValue);
        SetAntiAliasing(antialiasingValue);

        PlayerPrefs.SetInt(PreferenceKey.QualitySettingPreference.Value, qualityDropdown.value);
        PlayerPrefs.Save();
    }

    public void ExitGame()
    {
        Application.Quit();
    }

    private int LoadPreferenceInt(PreferenceKey key, int defaultValue)
    {
        if (PlayerPrefs.HasKey(key.Value))
        {
            int value = PlayerPrefs.GetInt(key.Value);
            Debug.Log($"Player preference found for {key.Value}: {value}");
            return value;
        }
        Debug.Log($"No player preference found for {key.Value}. Using default: {defaultValue}");
        return defaultValue;
    }

    private string LoadPreferenceString(PreferenceKey key, string defaultValue)
    {
        if (PlayerPrefs.HasKey(key.Value))
        {
            string value = PlayerPrefs.GetString(key.Value);
            Debug.Log($"Player preference found for {key.Value}: {value}");
            return value;
        }
        Debug.Log($"No player preference found for {key.Value}. Using default: {defaultValue}");
        return defaultValue;
    }

    public void LoadSettings(int currentResolutionIndex)
    {
        Debug.Log($"Loading settings. Default resolution index is {currentResolutionIndex}");
        qualityDropdown.value = LoadPreferenceInt(PreferenceKey.QualitySettingPreference, 3);
        resolutionDropdown.value = LoadPreferenceInt(PreferenceKey.ResolutionPreference, currentResolutionIndex);
        toggleFullscreen.isOn = Convert.ToBoolean(LoadPreferenceInt(PreferenceKey.FullscreenPreference, 0));
        string scenario_key = LoadPreferenceString(PreferenceKey.ScenarioPreference, "");
        if (scenarioIndex.ContainsKey(scenario_key))
        {
            scenarioDropdown.value = scenarioIndex[scenario_key];
        }
        else
        {
            Debug.LogWarning($"Scenario {scenario_key} not found in scenario index");
            scenarioDropdown.value = 0;
        }
        enableSimulatedCamerasToggle.isOn = Convert.ToBoolean(LoadPreferenceInt(PreferenceKey.SimulatedCameraPreference, 0));
    }

    void RemoteScenarioSelectionCallback(StringMsg msg)
    {
        Debug.Log($"Received remote scenario selection: {msg.data}");
        if (scenarioIndex.ContainsKey(msg.data))
        {
            scenarioDropdown.value = scenarioIndex[msg.data];
            scenarioDropdown.RefreshShownValue();
            SetScenario(scenarioDropdown.value);
            ShowHideSettingsPanel(false);
            sceneManager.GetPauseManager().SetPause(false);
        }
        else
        {
            Debug.LogWarning($"Scenario {msg.data} not found in scenario index");
        }
    }

    IEnumerator<object> PublishScenarioListCallback()
    {
        while (true)
        {
            ros.Publish(scenarioListTopic, new LabelsMsg { labels = scenarios.ToArray() });
            yield return new WaitForSecondsRealtime(1);
        }
    }

    IEnumerator<object> UpdateFpsCounter()
    {
        while (true)
        {
            fpsReadout.text = $"FPS: {fpsCounter.GetFps():  0.00}";
            yield return new WaitForSecondsRealtime(0.1f);
        }
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Escape))
        {
            ShowHideSettingsPanel(!isShown);
        }
    }
}
