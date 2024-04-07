using UnityEngine;

public class DisplayReadoutManager : MonoBehaviour
{
    [SerializeField] Material[] materials;
    MeshRenderer meshRenderer;

    int currentMaterialIndex = 0;
    bool enableClicks = true;

    void Start()
    {
        meshRenderer = GetComponent<MeshRenderer>();
        meshRenderer.material = materials[currentMaterialIndex];
    }

    void Update()
    {

    }

    public void SetEnableClicks(bool enabled)
    {
        enableClicks = enabled;
    }

    void OnMouseUpAsButton()
    {
        Debug.Log("Clicked on display readout");
        if (!enableClicks)
        {
            return;
        }
        currentMaterialIndex = (currentMaterialIndex + 1) % materials.Length;
        meshRenderer.material = materials[currentMaterialIndex];
    }
}
