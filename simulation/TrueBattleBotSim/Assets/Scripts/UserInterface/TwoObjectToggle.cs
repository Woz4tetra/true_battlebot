using UnityEngine;

public class TwoObjectToggle : MonoBehaviour
{
    [SerializeField] GameObject object1;
    [SerializeField] GameObject object2;

    public void SetObjectActive(bool object1Active)
    {
        object1.SetActive(object1Active);
        object2.SetActive(!object1Active);
    }
}
