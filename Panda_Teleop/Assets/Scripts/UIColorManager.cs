using UnityEngine;
using UnityEngine.UI;


public class UIColorManager : MonoBehaviour
{
    [Header("UI Elements")]
    [Tooltip("The GameObject containing the color wheel UI.")]
    public GameObject colorWheelObject;

    [Tooltip("The Image component that will display the selected color.")]
    public Image colorDisplayImage;

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
        if (colorDisplayImage != null)
        {
            // Update the display image with the new color.
            colorDisplayImage.color = newColor;
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

    // Update is called once per frame
    void Update()
    {
        
    }
}
