using UnityEngine;

public class FPSCounter : MonoBehaviour
{
    private float[] samples;
    [SerializeField] private int rollingAverageWindow = 30;
    private int averageCounter = 0;
    private float currentAveragedFps;

    void Awake()
    {
        samples = new float[rollingAverageWindow];
    }
    void Update()
    {
        // Sample
        float currentRate = 1.0f / Time.unscaledDeltaTime;
        samples[averageCounter] = currentRate;

        // Average
        float sum = 0.0f;

        foreach (var frameRate in samples)
        {
            sum += frameRate;
        }

        currentAveragedFps = sum / rollingAverageWindow;
        averageCounter = (averageCounter + 1) % rollingAverageWindow;
    }

    public float GetFps()
    {
        return currentAveragedFps;
    }
}
