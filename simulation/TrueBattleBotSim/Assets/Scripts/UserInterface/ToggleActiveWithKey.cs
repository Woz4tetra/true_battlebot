using UnityEngine;

public class ToggleActiveWithKey : MonoBehaviour
{
    [SerializeField] KeyCode toggleKey = KeyCode.Space;
    [SerializeField] GameObject targetObject;

    void Update()
    {
        if (Input.GetKeyDown(toggleKey))
        {
            targetObject.SetActive(!targetObject.activeSelf);
        }
    }
}