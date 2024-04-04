using UnityEngine;

public class FieldModelController : MonoBehaviour
{
    [SerializeField] GameObject fieldRoof = null;
    [SerializeField] GameObject cameraObject = null;
    [SerializeField] float transitionFudgeFactor = 0.09f;

    void Start()
    {
        fieldRoof.SetActive(true);
    }

    void Update()
    {
        fieldRoof.SetActive(!IsCameraAboveRoof());
    }

    bool IsCameraAboveRoof()
    {
        return cameraObject.transform.position.y > GetFieldY();
    }

    float GetFieldY()
    {
        return fieldRoof.GetComponent<MeshRenderer>().bounds.size.y + fieldRoof.transform.position.y - transitionFudgeFactor;
    }
}