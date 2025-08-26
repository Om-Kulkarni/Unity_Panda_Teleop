using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.IO;
using System;
using System.IO;
using System.Collections.Generic;

// A single, unified class to hold all data for one trial.
// This makes linking the data automatic.
[System.Serializable]
public class TrialData
{
    public string trialId; // Unique ID to identify this specific trial
    public string timestamp;
    public float timeTakenSeconds;

    // Ground Truth (captured when robot picks up the cube)
    public Color trueColor;
    public string trueSize;  // Changed to string to store "small", "medium", or "large"
    public string trueTexture;

    // User Selection (captured when user clicks 'Save')
    public Color userSelectedColor;
    public string userSelectedSize;  // Changed to string to store "small", "medium", or "large"
    public string userSelectedTexture;
}

// This class is the root object for the entire JSON file.
// It holds session-wide data AND the list of all trials.
[System.Serializable]
public class SessionData
{
    // Session-wide data
    public float totalSessionTimeSeconds;

    // List of individual trial data
    public List<TrialData> allTrials = new List<TrialData>();
}

public class DatabaseManager : MonoBehaviour
{
    [Header("UI References")]
    public Image colorDisplay;
    public TMP_Dropdown sizeDropdown;  // Changed from Slider to Dropdown for size categories
    public TMP_Dropdown textureDropdown;

    // Size category thresholds
    private const float SMALL_SIZE = 0.03f;
    private const float MEDIUM_SIZE = 0.04f;
    private const float LARGE_SIZE = 0.05f;

    // Helper method to convert actual size to category
    private string SizeToCategory(float size)
    {
        if (size <= SMALL_SIZE) return "small";
        if (size <= MEDIUM_SIZE) return "medium";
        return "large";
    }

    // Helper method to convert category to actual size
    private float CategoryToSize(string category)
    {
        switch (category.ToLower())
        {
            case "small": return SMALL_SIZE;
            case "medium": return MEDIUM_SIZE;
            case "large": return LARGE_SIZE;
            default: return MEDIUM_SIZE;
        }
    }

    [Header("File Naming")]
    public string participantName = "P01";  // Default value, can be changed in Inspector

    private string saveFileName;
    private TrialData activeTrial; // Holds the current trial data in memory

    // Variables to track time
    private float trialStartTime; // Time when the current cube was picked up.
    private float sessionStartTime; // Time when the VERY FIRST cube was picked up.

    private string dataFolderPath;

    void Awake()
    {
        saveFileName = $"{participantName}_TrialData_{DateTime.Now:yyyy-MM-dd_HH-mm}.json";
        
        // Set up the data folder path in the workspace root
        dataFolderPath = Path.Combine(Application.dataPath, "../../data");
        
        // Create the data directory if it doesn't exist
        if (!Directory.Exists(dataFolderPath))
        {
            Directory.CreateDirectory(dataFolderPath);
        }
    }

    /// <summary>
    /// STEP 1: Called by the robot script when a cube is picked up.
    /// Captures ground truth and prepares the entry.
    /// </summary>
    /// <param name="groundTruthCube">The cube GameObject that the robot grasped.</param>
    public void StartNewDataEntry(GameObject groundTruthCube)
    {
        if (groundTruthCube == null)
        {
            Debug.LogError("StartNewDataEntry called, but groundTruthCube was null.");
            return;
        }

        // --- Get cube properties first ---
        Renderer cubeRenderer = groundTruthCube.GetComponent<Renderer>();
        Material cubeMaterial = cubeRenderer.material;
        float metallicValue = cubeMaterial.GetFloat("_Metallic");
        string textureName;

        // --- NEW LOGIC: Determine texture name based on metallic value ---
        if (metallicValue < 0.2f)
        {
            textureName = "Rough";
        }
        else if (metallicValue > 0.8f)
        {
            textureName = "Metallic";
        }
        else
        {
            textureName = "Standard"; // A default for values in between
        }

        // Start the timers using Unity's Time.time
        trialStartTime = Time.time;

        // If this is the first cube of the session, start the main session timer.
        if (sessionStartTime == 0f)
        {
            sessionStartTime = Time.time;
        }

        // Create a new data entry in memory
        activeTrial = new TrialData
        {
            trialId = Guid.NewGuid().ToString(), // Generate a unique ID for this trial
            timestamp = DateTime.UtcNow.ToString("o"), // ISO 8601 format

            // Populate ground truth fields
            trueColor = cubeMaterial.color,
            trueSize = SizeToCategory(groundTruthCube.transform.localScale.x),  // Convert actual size to category
            trueTexture = textureName
        };

        Debug.Log($"Started new data entry with ID: {activeTrial.trialId}. Waiting for user input.");
    }

    /// <summary>
    /// STEP 2: Called by the UI Save Button's OnClick event.
    /// Completes the entry with user data and saves everything to the JSON file.
    /// </summary>
    public void ConfirmAndSaveData()
    {
        // Check if a cube has been picked up first
        if (activeTrial == null)
        {
            Debug.LogError("Save button clicked, but no cube has been picked up yet. Please start a new entry first.");
            return;
        }

        //  Calculate the time durations
        float timeTakenForTrial = Time.time - trialStartTime;
        float totalSessionTime = Time.time - sessionStartTime;

        Debug.Log($"Time for this trial: {timeTakenForTrial:F2}s. Total session time: {totalSessionTime:F2}s.");

        // Populate the user selection fields of the active trial
        activeTrial.userSelectedColor = colorDisplay.color;
        activeTrial.userSelectedSize = sizeDropdown.options[sizeDropdown.value].text.ToLower();  // Get selected size category
        activeTrial.userSelectedTexture = textureDropdown.options[textureDropdown.value].text;

        // --- Now, save the completed trial data to the file ---
        string filePath = Path.Combine(dataFolderPath, saveFileName);
        SessionData dataList = new SessionData();

        if (File.Exists(filePath))
        {
            string json = File.ReadAllText(filePath);
            dataList = JsonUtility.FromJson<SessionData>(json);
        }

        dataList.allTrials.Add(activeTrial);
        string updatedJson = JsonUtility.ToJson(dataList, true);
        File.WriteAllText(filePath, updatedJson);

        Debug.Log($"Successfully saved complete data for trial ID: {activeTrial.trialId}");

        // Reset UI elements to default values
        colorDisplay.color = Color.white;  // Reset color to white
        sizeDropdown.value = 0;           // Reset size to first option (small)
        textureDropdown.value = 0;        // Reset to first option (assuming "Rough" is first)

        // Reset the active trial to null, ready for the next cycle
        activeTrial = null;
    }

    // HELPER METHODS to reduce code duplication 
    private SessionData LoadSessionData()
    {
        string filePath = Path.Combine(dataFolderPath, saveFileName);
        if (File.Exists(filePath))
        {
            string json = File.ReadAllText(filePath);
            return JsonUtility.FromJson<SessionData>(json);
        }
        return new SessionData(); // Return a new, empty object if file doesn't exist
    }

    private void SaveSessionData(SessionData data)
    {
        string filePath = Path.Combine(dataFolderPath, saveFileName);
        string updatedJson = JsonUtility.ToJson(data, true);
        File.WriteAllText(filePath, updatedJson);
    }
}