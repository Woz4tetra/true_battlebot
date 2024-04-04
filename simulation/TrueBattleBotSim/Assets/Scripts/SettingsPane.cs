using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class SettingsPane : MonoBehaviour
{
    [SerializeField] TMP_Dropdown resolutionDropdown;
    [SerializeField] TMP_Dropdown qualityDropdown;
    [SerializeField] Toggle toggleFullscreen;
    [SerializeField] GameObject settingsPanel;
    [SerializeField] GameObject enterSettingsPanel;
    [SerializeField] CameraController cameraController;
    Resolution[] resolutions;
    int textureValue = 0;
    int antialiasingValue = 0;

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
        public static PreferenceKey TextureQualityPreference { get { return new PreferenceKey("TextureQualityPreference"); } }
        public static PreferenceKey AntiAliasingPreference { get { return new PreferenceKey("AntiAliasingPreference"); } }
        public static PreferenceKey FullscreenPreference { get { return new PreferenceKey("FullscreenPreference"); } }

        public override string ToString()
        {
            return Value;
        }
    }

    // Start is called before the first frame update
    void Start()
    {
        resolutionDropdown.ClearOptions();
        List<string> options = new List<string>();
        resolutions = Screen.resolutions;
        int currentResolutionIndex = 0;

        for (int i = 0; i < resolutions.Length; i++)
        {
            string option = resolutions[i].width + " x " + resolutions[i].height;
            options.Add(option);
            if (resolutions[i].width == Screen.currentResolution.width
                    && resolutions[i].height == Screen.currentResolution.height)
                currentResolutionIndex = i;
        }

        resolutionDropdown.AddOptions(options);
        resolutionDropdown.RefreshShownValue();
        LoadSettings(currentResolutionIndex);
        ShowHideSettingsPanel(settingsPanel.activeSelf);
    }

    public void SetFullscreen(bool isFullscreen)
    {
        Screen.fullScreen = isFullscreen;
    }

    public void SetResolution(int resolutionIndex)
    {
        Resolution resolution = resolutions[resolutionIndex];
        Screen.SetResolution(resolution.width, resolution.height, Screen.fullScreen);
        ScalePanelToResolution();
    }

    private void SetTextureQuality(int textureIndex)
    {
        QualitySettings.globalTextureMipmapLimit = textureIndex;
    }
    private void SetAntiAliasing(int aaIndex)
    {
        QualitySettings.antiAliasing = aaIndex;
    }

    public void ShowHideSettingsPanel(bool show)
    {
        settingsPanel.SetActive(show);
        enterSettingsPanel.gameObject.SetActive(!show);
        cameraController.EnableControls(!show);
    }


    private void ScalePanelToResolution()
    {
        float targetWidth = Screen.width;
        float targetHeight = Screen.height;
        float sourceWidth = 1920;
        float sourceHeight = 1080;
        float scale = Mathf.Min(targetHeight / sourceHeight, targetWidth / sourceWidth);

        settingsPanel.transform.localScale = new Vector3(scale, scale, scale);
        enterSettingsPanel.transform.localScale = new Vector3(scale, scale, scale);
    }


    public void SetQuality(int qualityIndex)
    {
        QualitySettings.SetQualityLevel(qualityIndex);
        TextureQualityPreset quality = (TextureQualityPreset)qualityIndex;
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

        qualityDropdown.value = (int)qualityIndex;
        SetTextureQuality(textureValue);
        SetAntiAliasing(antialiasingValue);
    }

    public void ExitGame()
    {
        Application.Quit();
    }
    public void SaveSettings()
    {
        PlayerPrefs.SetInt(PreferenceKey.QualitySettingPreference.Value, qualityDropdown.value);
        PlayerPrefs.SetInt(PreferenceKey.ResolutionPreference.Value, resolutionDropdown.value);
        PlayerPrefs.SetInt(PreferenceKey.TextureQualityPreference.Value, textureValue);
        PlayerPrefs.SetInt(PreferenceKey.AntiAliasingPreference.Value, antialiasingValue);
        PlayerPrefs.SetInt(PreferenceKey.FullscreenPreference.Value, Convert.ToInt32(Screen.fullScreen));
    }

    private int LoadPreference(PreferenceKey key, int defaultValue)
    {
        if (PlayerPrefs.HasKey(key.Value))
        {
            return PlayerPrefs.GetInt(key.Value);
        }
        return defaultValue;
    }

    public void LoadSettings(int currentResolutionIndex)
    {
        qualityDropdown.value = LoadPreference(PreferenceKey.QualitySettingPreference, 3);
        resolutionDropdown.value = LoadPreference(PreferenceKey.ResolutionPreference, currentResolutionIndex);
        textureValue = LoadPreference(PreferenceKey.TextureQualityPreference, 0);
        antialiasingValue = LoadPreference(PreferenceKey.AntiAliasingPreference, 1);
        bool fullScreen = Convert.ToBoolean(LoadPreference(PreferenceKey.FullscreenPreference, 1));
        toggleFullscreen.isOn = fullScreen;

        SetQuality(qualityDropdown.value);
        SetResolution(resolutionDropdown.value);
        SetTextureQuality(textureValue);
        SetAntiAliasing(antialiasingValue);
        Screen.fullScreen = fullScreen;
    }

    // Update is called once per frame
    void Update()
    {

    }
}
