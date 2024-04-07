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
        bool paused = pauseManager.IsPaused();
        SetPause(paused);
    }

    private void SetPause(bool pausedState)
    {
        icon.sprite = pausedState ? playIcon : pauseIcon;
        Debug.Log($"Setting pause icon to {icon.sprite}");
    }

    public void OnClick()
    {
        bool pauseState = !pauseManager.IsPaused();
        SetPause(pauseState);
        pauseManager.SetPause(pauseState);
    }
}