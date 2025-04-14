using UnityEngine;

public class PauseManager : MonoBehaviour
{
    [SerializeField] bool isPaused = true;
    [SerializeField] float spamCooldown = 0.2f;

    float lastSetTime = float.NegativeInfinity;
    float configTimeScale = 1.0f;

    public bool IsPaused()
    {
        return isPaused;
    }

    public void SetTimeScale(float timeScale)
    {
        configTimeScale = timeScale;
    }

    public bool SetPause(bool paused)
    {
        float now = Time.unscaledTime;
        if (now - lastSetTime < spamCooldown)
        {
            Debug.LogWarning("Spam detected, ignoring pause/play request");
            return false;
        }
        lastSetTime = now;
        Debug.Log($"Set pause to {paused}");
        isPaused = paused;
        Time.timeScale = paused ? 0.0f : configTimeScale;
        return true;
    }
}