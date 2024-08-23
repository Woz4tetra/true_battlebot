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
    [SerializeField] Toggle toggleSpotlight;
    [SerializeField] TMP_Text fpsReadout;
    [SerializeField] GameObject settingsPanel;
    [SerializeField] GameObject settingsBackground;
    [SerializeField] GameObject enterSettingsPanel;
    [SerializeField] GameObject displayReadout;
    [SerializeField] FPSCounter fpsCounter;
    [SerializeField] CameraController cameraController;
    [SerializeField] FieldManager sceneManager;
    [SerializeField] string remoteScenarioSelectionTopic = "simulation/scenario_selection";
    [SerializeField] string addConfigurationTopic = "simulation/add_configuration";
    [SerializeField] string scenarioListTopic = "simulation/scenarios";
    [SerializeField] RestartButton restartButton;
    [SerializeField] PlayPauseButton playPauseButton;
    [SerializeField] TwoObjectToggle spotlightToggleManager;

    DisplayReadoutManager displayReadoutManager;

    List<Resolution> resolutions = new List<Resolution>();
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
        public static PreferenceKey SpotlightPreference { get { return new PreferenceKey("SpotlightPreference"); } }

        public override string ToString()
        {
            return Value;
        }
    }

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<StringMsg>(remoteScenarioSelectionTopic, RemoteScenarioSelectionCallback);
        ros.Subscribe<ConfigureSimulationMsg>(addConfigurationTopic, AddConfigurationCallback);
        ros.RegisterPublisher<LabelsMsg>(scenarioListTopic);

        resolutionDropdown.ClearOptions();
        List<string> options = new List<string>();
        Resolution[] foundResolutions = Screen.resolutions;
        int currentResolutionIndex = -1;

        float commonRatio = (float)commonWidth / commonHeight;
        int resolutionIndex = 0;

        for (int index = 0; index < foundResolutions.Length; index++)
        {
            float ratio = (float)foundResolutions[index].width / foundResolutions[index].height;
            if (!Mathf.Approximately(ratio, commonRatio))
            {
                continue;
            }
            string option = foundResolutions[index].width + " x " + foundResolutions[index].height;
            options.Add(option);
            resolutions.Add(foundResolutions[index]);
            resolutionIndex++;
            if (currentResolutionIndex != -1)
            {
                continue;
            }
            if (foundResolutions[index].width == commonWidth && foundResolutions[index].height == commonHeight)
            {
                currentResolutionIndex = resolutionIndex;
                Debug.Log(
                    $"Common resolution found: ({commonWidth} x {commonHeight}). " +
                    $"Labeling as index {currentResolutionIndex}"
                );
            }
            else if (foundResolutions[index].width == Screen.currentResolution.width
                    && foundResolutions[index].height == Screen.currentResolution.height)
            {
                currentResolutionIndex = resolutionIndex;
                Debug.Log(
                    $"Current resolution found: ({commonWidth} x {commonHeight}). " +
                    $"Labeling as index {currentResolutionIndex}"
                );
            }
        }
        if (currentResolutionIndex == -1)
        {
            currentResolutionIndex = resolutions.Count - 1;
        }

        scenarios = new List<string>(ConfigManager.GetScenarioNames());
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
    public void SetSpotlightCallback(bool isSpotlight)
    {
        SetSpotlight(isSpotlight);
    }

    private void SetFullscreen(bool isFullscreen)
    {
        Debug.Log($"Setting fullscreen to {isFullscreen}");
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
        sceneManager.LoadScenarioByName(scenarioName);
    }

    private void SetSpotlight(bool isSpotlight)
    {
        Debug.Log($"Setting spotlight to {isSpotlight}");
        PlayerPrefs.SetInt(PreferenceKey.SpotlightPreference.Value, Convert.ToInt32(isSpotlight));
        PlayerPrefs.Save();
        spotlightToggleManager.SetObjectActive(isSpotlight);
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
        Debug.Log($"Show settings panel: {show}");
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
        if (sceneManager.GetPauseManager().SetPause(paused))
        {
            playPauseButton.SetPause(paused);
        }
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
        toggleSpotlight.isOn = Convert.ToBoolean(LoadPreferenceInt(PreferenceKey.SpotlightPreference, 1));
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
        SetFullscreen(toggleFullscreen.isOn);
        SetSpotlight(toggleSpotlight.isOn);
    }

    void RemoteScenarioSelectionCallback(StringMsg msg)
    {
        Debug.Log($"Received remote scenario selection: {msg.data}");
        if (scenarioIndex.ContainsKey(msg.data))
        {
            scenarioDropdown.value = scenarioIndex[msg.data];
            scenarioDropdown.RefreshShownValue();
            SetScenario(scenarioDropdown.value);
            wasPausedWhenSettingsOpened = false;
            ShowHideSettingsPanel(false);
        }
        else
        {
            Debug.LogWarning($"Scenario {msg.data} not found in scenario index");
        }
    }

    void AddConfigurationCallback(ConfigureSimulationMsg msg)
    {
        UpdateConfigurationManager(msg);
        AddToScenarioDropdown(msg.scenario.name);
    }

    void UpdateConfigurationManager(ConfigureSimulationMsg msg)
    {
        Debug.Log($"Adding scenario {msg.scenario.name} to configuration manager");
        ConfigManager.AddScenarioFromJson(msg.scenario.name, msg.scenario.json_data);
        for (int i = 0; i < msg.objectives.Length; i++)
        {
            string objectiveName = msg.objectives[i].name;
            Debug.Log($"Adding objective {objectiveName} to configuration manager");
            ConfigManager.AddObjectiveFromJson(objectiveName, msg.objectives[i].json_data);
        }
    }

    void AddToScenarioDropdown(string scenarioName)
    {
        if (scenarios.Contains(scenarioName))
        {
            return;
        }
        Debug.Log($"Adding scenario {scenarioName} to dropdown");
        scenarios.Add(scenarioName);
        scenarioIndex[scenarioName] = scenarios.Count - 1;
        scenarioDropdown.AddOptions(new List<string> { scenarioName });
        scenarioDropdown.RefreshShownValue();
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

        if (!isShown)
        {
            if (Input.GetKeyDown(KeyCode.Space) || playPauseButton.WasClicked())
            {
                SetPause(!sceneManager.GetPauseManager().IsPaused());
            }

            if (Input.GetKeyDown(KeyCode.R))
            {
                restartButton.RestartScenario();
            }
        }
    }
}
