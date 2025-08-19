using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class GridColorController : MonoBehaviour
{
    [Header("UI Grid Elements")]
    // List of Image components representing the grid cells. Assign these in the Unity Editor.
    public List<Image> gridImages = new List<Image>();

    [Header("Color Configuration")]
    // List of hex color codes to apply to the grid cells. Each string should be in #RRGGBB format.
    private List<string> hexColorCodes = new List<string>
    {
        "#5C89FF", // Blue
        "#00D117", // Green
        "#C5E620", // Light Green
        "#BE8FFF", // Light Purple
        "#FFA600", // Orange
        "#FF82BB", // Pink
        "#CF66FF", // Purple
        "#FF0000", // Red
        "#11CFC6"  // Turquoise
    };

    // This function is called when the script instance is being loaded.
    void Awake()
    {
    // Automatically apply colors to the grid when the script instance loads.
    ApplyColorsToGrid();
    }

    /// <summary>
    /// Loops through the images and applies the specified colors.
    /// </summary>
    public void ApplyColorsToGrid()
    {
        // Shuffle the hexColorCodes list to randomize color assignment without repetition.
        List<string> shuffledColors = new List<string>(hexColorCodes);
        int n = shuffledColors.Count;
        for (int i = n - 1; i > 0; i--)
        {
            int j = Random.Range(0, i + 1);
            // Swap shuffledColors[i] and shuffledColors[j]
            string temp = shuffledColors[i];
            shuffledColors[i] = shuffledColors[j];
            shuffledColors[j] = temp;
        }

        // Determine how many colors/images to apply (use the smaller list size).
        int itemCount = Mathf.Min(gridImages.Count, shuffledColors.Count);
        for (int i = 0; i < itemCount; i++)
        {
            // Get the current image and corresponding shuffled hex color code.
            Image image = gridImages[i];
            string hex = shuffledColors[i];
            // Try to parse the hex string into a Unity Color object.
            if (ColorUtility.TryParseHtmlString(hex, out Color newColor))
            {
                // If successful, apply the color to the image.
                image.color = newColor;
            }
            else
            {
                // If parsing fails, log an error with details.
                Debug.LogError($"Invalid hex color code: '{hex}' at index {i}. Please check the format (e.g., #RRGGBB).");
            }
        }
    }
}
