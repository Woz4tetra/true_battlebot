using UnityEngine;

public class TransformFrame : MonoBehaviour {
    [SerializeField] private string frameId = "";

    public string GetFrameId() {
        return frameId;
    }
}