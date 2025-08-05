using UnityEngine;
using UnityEngine.UI;
using TMPro; // Required for TextMeshPro Dropdowns
using System.IO; // Required for file operations
using System; // Required for DateTime

public class DatabaseManager : MonoBehaviour
{
    [Header("UI References")]
    [Tooltip("The Image component showing the selected color.")]
    public Image colorDisplay;

    [Tooltip("The Slider component for selecting the size.")]
    public Slider sizeSlider;

    [Tooltip("The Dropdown for selecting the texture.")]
    public TMP_Dropdown textureDropdown;

    [Tooltip("The Dropdown for selecting the quality.")]
    public TMP_Dropdown qualityDropdown;

    private string saveFileName;
    // Start is called before the first frame update

        void Awake()
    {
        // Filename that includes the current date, hours, and minutes to prevent overwriting.
        // Example: "CubeData_2025-08-05_16-04.json"
        saveFileName = $"CubeData_{DateTime.Now:yyyy-MM-dd_HH-mm}.json";
    }

    /// <summary>
    /// Gathers data from the UI, loads existing data from the JSON file,
    /// adds the new data, and saves it all back to the file.
    /// This method should be called by the 'Save' button's OnClick event.
    /// FilePath: C:\Users\<YourUsername>\AppData\LocalLow\<CompanyName>\<ProductName>
    /// </summary>
    public void SaveData()
    {
        // 1. Get the full path to the save file.
        string filePath = Path.Combine(Application.persistentDataPath, saveFileName);
        Debug.Log($"Saving data to: {filePath}");

        // 2. Load the existing data from the file, or create a new list if the file doesn't exist.
        CubeDataList dataList = new CubeDataList();
        if (File.Exists(filePath))
        {
            string json = File.ReadAllText(filePath);
            dataList = JsonUtility.FromJson<CubeDataList>(json);
        }

        // 3. Create a new CubeData object with the current UI values.
        CubeData newData = new CubeData
        {
            cubeColor = colorDisplay.color,
            cubeSize = sizeSlider.value,
            cubeTexture = textureDropdown.options[textureDropdown.value].text,
            cubeQuality = qualityDropdown.options[qualityDropdown.value].text
        };

        // 4. Add the new data to our list.
        dataList.allCubes.Add(newData);

        // 5. Convert the updated list back to JSON.
        string updatedJson = JsonUtility.ToJson(dataList, true); // 'true' for pretty print

        // 6. Write the JSON string to the file, overwriting the old one.
        File.WriteAllText(filePath, updatedJson);

        Debug.Log("Data saved successfully!");
    }
}
