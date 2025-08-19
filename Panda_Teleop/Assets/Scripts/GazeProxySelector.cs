using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;

/// <summary>
/// This script sits on a visualizer/cursor object. It uses physics trigger events
/// to detect when it's touching a UI Button's collider. When the select action
/// is performed, it "clicks" the button it is currently touching.
/// </summary>
public class GazeProxySelector : MonoBehaviour
{
    [Tooltip("The Input Action to trigger a UI click.")]
    public InputActionReference selectAction;

    // A private variable to keep track of the button we are currently touching.
    private Button currentButton = null;

    private void OnEnable()
    {
        if (selectAction != null)
        {
            selectAction.action.Enable();
            selectAction.action.performed += OnSelectPerformed;
        }
    }

    private void OnDisable()
    {
        if (selectAction != null)
        {
            selectAction.action.performed -= OnSelectPerformed;
            selectAction.action.Disable();
        }
    }

    /// <summary>
    /// Called by the Input System when the select button is pressed.
    /// </summary>
    private void OnSelectPerformed(InputAction.CallbackContext context)
    {
        // If we are currently touching a button, invoke its onClick event.
        if (currentButton != null)
        {
            Debug.Log("ProxyClicker: Clicking button '" + currentButton.name + "'");
            currentButton.onClick.Invoke();
        }
    }

    /// <summary>
    /// Called by the Unity physics engine when this object's collider enters another trigger collider.
    /// </summary>
    private void OnTriggerEnter(Collider other)
    {
        // When we start touching a new object, check if it has a Button component.
        Button button = other.GetComponentInParent<Button>();
        if (button != null)
        {
            Debug.Log("ProxyClicker: Hovering over button '" + button.name + "'");
            currentButton = button;
        }
    }

    /// <summary>
    /// Called by the Unity physics engine when this object's collider exits another trigger collider.
    /// </summary>
    private void OnTriggerExit(Collider other)
    {
        // If we stop touching the button we were previously hovering over, clear the reference.
        if (currentButton != null && other.gameObject == currentButton.gameObject)
        {
            Debug.Log("ProxyClicker: Stopped hovering over button '" + currentButton.name + "'");
            currentButton = null;
        }
    }
}
