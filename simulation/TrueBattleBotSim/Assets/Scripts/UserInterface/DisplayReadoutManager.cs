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
        if (Input.GetKeyDown(KeyCode.Y))
        {
            NextView();
        }
    }

    void NextView()
    {
        currentMaterialIndex = (currentMaterialIndex + 1) % materials.Length;
        meshRenderer.material = materials[currentMaterialIndex];
    }

    public void SetEnableClicks(bool enabled)
    {
        enableClicks = enabled;
    }

    void OnMouseUp()
    {
        Debug.Log("Clicked on display readout");
        if (!enableClicks)
        {
            return;
        }
        NextView();
    }
}
