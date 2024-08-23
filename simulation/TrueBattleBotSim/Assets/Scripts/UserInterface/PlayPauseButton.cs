using UnityEngine;
using UnityEngine.UI;

public class PlayPauseButton : MonoBehaviour
{
    [SerializeField] Sprite playIcon;
    [SerializeField] Sprite pauseIcon;

    Image icon;
    Button button;
    bool wasClicked = false;

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
        wasClicked = true;
    }

    public bool WasClicked()
    {
        if (wasClicked)
        {
            wasClicked = false;
            return true;
        }
        return false;
    }
}