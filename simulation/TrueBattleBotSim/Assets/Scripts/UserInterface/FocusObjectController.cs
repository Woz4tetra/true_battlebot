using UnityEngine;

public class FocusObjectController : MonoBehaviour
{
    [SerializeField] GameObject focusObject = null;
    private bool prevButtonState = false;
    private Vector2 prevClickPosition = Vector2.zero;
    private bool enableControls = true;

    void Start()
    {
        if (focusObject == null)
        {
            focusObject = new GameObject();
            focusObject.transform.position = Vector3.zero;
        }
    }

    private bool GetMiddleMouseDown()
    {
        return Input.GetButton("Fire3");
    }

    public void EnableControls(bool enable)
    {
        enableControls = enable;
    }


    private Vector2 GetMovementVector()
    {
        bool buttonState = GetMiddleMouseDown();
        Vector2 mousePosition = Input.mousePosition;
        if (buttonState != prevButtonState)
        {
            prevButtonState = buttonState;
            if (buttonState)
            {
                prevClickPosition = mousePosition;
            }
        }

        if (buttonState)
        {
            Vector2 rawDirection = mousePosition - prevClickPosition;
            Vector2 scaledDirection = new Vector2(rawDirection.x / Screen.width, rawDirection.y / Screen.height);
            return Vector2.ClampMagnitude(scaledDirection, 1.0f);
        }
        else
        {
            return Vector2.zero;
        }
    }

    void Update()
    {
        if (!enableControls)
        {
            return;
        }
        Vector2 movement = GetMovementVector();
        if (movement.magnitude > 0.01f)
        {
            focusObject.transform.Translate(movement.x, 0, movement.y);
            transform.Translate(movement.x, 0, movement.y);
        }
    }
}