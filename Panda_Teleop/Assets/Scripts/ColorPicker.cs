using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;

public class ColorPicker : MonoBehaviour, IPointerClickHandler
{
    [Header("References")]
    [Tooltip("Reference to the UIColorManager to send the selected color to.")]
    public UIColorManager uiColorManager;

    private Image colorWheelImage;
    private Texture2D colorWheelTexture;

    void Awake()
    {
        colorWheelImage = GetComponent<Image>();
        // Get the texture from the sprite.
        // IMPORTANT: The texture must have "Read/Write Enabled" checked in its import settings.
        colorWheelTexture = colorWheelImage.sprite.texture;

        if (uiColorManager == null)
        {
            Debug.LogError("UIColorManager reference is not set on the ColorPicker script.");
        }
    }

    /// <summary>
    /// This method is called when the user clicks on this UI element.
    /// </summary>
    public void OnPointerClick(PointerEventData eventData)
    {
        // Convert the click position to the local space of the RectTransform.
        RectTransformUtility.ScreenPointToLocalPointInRectangle(
            (RectTransform)transform,
            eventData.position,
            eventData.pressEventCamera,
            out Vector2 localPoint
        );

        // Get the RectTransform of the image.
        RectTransform imageRect = (RectTransform)transform;
        Rect rect = imageRect.rect;

        // Normalize the local point to be within the 0-1 range of the texture.
        float normalizedX = (localPoint.x - rect.x) / rect.width;
        float normalizedY = (localPoint.y - rect.y) / rect.height;

        // Convert the normalized coordinates to pixel coordinates.
        int pixelX = (int)(normalizedX * colorWheelTexture.width);
        int pixelY = (int)(normalizedY * colorWheelTexture.height);

        // Get the color from the texture at the specified pixel.
        Color selectedColor = colorWheelTexture.GetPixel(pixelX, pixelY);

        // Pass the selected color to the manager.
        if (uiColorManager != null && selectedColor.a > 0) // Only select if the pixel is not translucent
        {
            uiColorManager.SetColor(selectedColor);
        }
    }
}
