using UnityEngine;

public class PauseManager : MonoBehaviour
{
    bool isPaused = false;

    public bool IsPaused()
    {
        return isPaused;
    }

    public void SetPause(bool paused)
    {
        Debug.Log($"Set pause to {paused}");
        isPaused = paused;
        Time.timeScale = paused ? 0 : 1;

    }
}