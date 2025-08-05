using UnityEngine;
using UnityEngine.UI;


public class UIColorManager : MonoBehaviour
{
    [Header("UI Elements")]
    [Tooltip("The GameObject containing the color wheel UI.")]
    public GameObject colorWheelObject;

        [Tooltip("The Image component that will display the selected color in ColorSelection UI.")]
        public Image colorDisplayImageColorSelection;

        [Tooltip("The Image component that will display the selected color in Database UI.")]
        public Image colorDisplayImageColorDatabase;

    // Start is called before the first frame update
    void Start()
    {
        // Ensure the color wheel is hidden at the start.
        if (colorWheelObject != null)
        {
            colorWheelObject.SetActive(false);
        }
    }

    /// <summary>
    /// Toggles the active state of the color wheel GameObject.
    /// This method is intended to be called by a UI Button's OnClick event.
    /// </summary>
    public void ToggleColorWheel()
    {
        if (colorWheelObject != null)
        {
            // Toggle the visibility of the color wheel.
            colorWheelObject.SetActive(!colorWheelObject.activeSelf);
        }
        else
        {
            Debug.LogError("Color Wheel GameObject is not assigned in the UIColorManager.");
        }
    }

    /// <summary>
    /// Sets the color of the display image and hides the color wheel.
    /// This method is called by the ColorPicker script when a color is selected.
    /// </summary>
    /// <param name="newColor">The color selected by the user.</param>
    public void SetColor(Color newColor)
    {
        if (colorDisplayImageColorSelection != null && colorDisplayImageColorDatabase != null)
        {
            // Update both display images with the new color.
            colorDisplayImageColorSelection.color = newColor;
            colorDisplayImageColorDatabase.color = newColor;
        }
        else
        {
            Debug.LogError("Color Display Image is not assigned in the UIColorManager.");
        }

        // Hide the color wheel after a selection is made.
        if (colorWheelObject != null)
        {
            colorWheelObject.SetActive(false);
        }
    }

    
}
