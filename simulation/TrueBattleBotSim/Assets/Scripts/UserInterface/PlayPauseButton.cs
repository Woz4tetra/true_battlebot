using UnityEngine;
using UnityEngine.UI;

public class PlayPauseButton : MonoBehaviour
{
    [SerializeField] PauseManager pauseManager;
    [SerializeField] Sprite playIcon;
    [SerializeField] Sprite pauseIcon;

    Image icon;
    Button button;

    void Start()
    {
        icon = transform.Find("Image").GetComponent<Image>();
        button = GetComponent<Button>();
        button.onClick.AddListener(OnClick);
    }

    public void SetPause(bool pausedState)
    {
        icon.sprite = pausedState ? playIcon : pauseIcon;
    }

    public void OnClick()
    {
        bool pauseState = !pauseManager.IsPaused();
        Debug.Log($"Setting pause icon to {pauseState}");
        pauseManager.SetPause(pauseState);
    }
}