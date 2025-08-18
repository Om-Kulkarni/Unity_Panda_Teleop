using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.IO;
using System;
using System.Collections.Generic;

// A single, unified class to hold all data for one trial.
// This makes linking the data automatic.
[System.Serializable]
public class TrialData
{
    public string trialId; // Unique ID to identify this specific trial
    public string timestamp;

    // Ground Truth (captured when robot picks up the cube)
    public Color trueColor;
    public Vector3 trueSize;
    public string trueTexture;
    public string trueQuality;

    // User Selection (captured when user clicks 'Save')
    public Color userSelectedColor;
    public float userSelectedSize;
    public string userSelectedTexture;
    public string userSelectedQuality;
}

// This class is the root object for the entire JSON file.
// It holds session-wide data AND the list of all trials.
[System.Serializable]
public class SessionData
{
    // Session-wide data
    public int totalSpikesOccurred;
    public int totalSpikesSaved;

    // List of individual trial data
    public List<TrialData> allTrials = new List<TrialData>();
}

public class DatabaseManager : MonoBehaviour
{
    [Header("UI References")]
    public Image colorDisplay;
    public Slider sizeSlider;
    public TMP_Dropdown textureDropdown;
    public TMP_Dropdown qualityDropdown;

    private string saveFileName;
    private TrialData activeTrial; // Holds the current trial data in memory

    void Awake()
    {
        saveFileName = $"TrialData_{DateTime.Now:yyyy-MM-dd_HH-mm}.json";
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

        // Create a new data entry in memory
        activeTrial = new TrialData
        {
            trialId = Guid.NewGuid().ToString(), // Generate a unique ID for this trial
            timestamp = DateTime.UtcNow.ToString("o"), // ISO 8601 format

            // Populate ground truth fields
            trueColor = groundTruthCube.GetComponent<Renderer>().material.color,
            trueSize = groundTruthCube.transform.localScale,
            trueTexture = "DummyTexture",
            trueQuality = "DummyQuality"
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

        // Populate the user selection fields of the active trial
        activeTrial.userSelectedColor = colorDisplay.color;
        activeTrial.userSelectedSize = sizeSlider.value;
        activeTrial.userSelectedTexture = textureDropdown.options[textureDropdown.value].text;
        activeTrial.userSelectedQuality = qualityDropdown.options[qualityDropdown.value].text;

        // --- Now, save the completed trial data to the file ---
        string filePath = Path.Combine(Application.persistentDataPath, saveFileName);
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

        // Reset the active trial to null, ready for the next cycle
        activeTrial = null;
    }

    /// <summary>
    /// STEP 3: Called by GraphController when the game stops.
    /// Updates the spike counts in the existing JSON file.
    /// </summary>
    public void UpdateSessionSpikeCounts(int occurredCount, int savedCount)
    {
        SessionData sessionData = LoadSessionData(); // Load whatever data already exists
        sessionData.totalSpikesOccurred = occurredCount;
        sessionData.totalSpikesSaved = savedCount;
        SaveSessionData(sessionData); // Save the updated object back to the file

        Debug.Log("Updated session spike counts in JSON file.");
    }

    // HELPER METHODS to reduce code duplication 
    private SessionData LoadSessionData()
    {
        string filePath = Path.Combine(Application.persistentDataPath, saveFileName);
        if (File.Exists(filePath))
        {
            string json = File.ReadAllText(filePath);
            return JsonUtility.FromJson<SessionData>(json);
        }
        return new SessionData(); // Return a new, empty object if file doesn't exist
    }

    private void SaveSessionData(SessionData data)
    {
        string filePath = Path.Combine(Application.persistentDataPath, saveFileName);
        string updatedJson = JsonUtility.ToJson(data, true);
        File.WriteAllText(filePath, updatedJson);
    }
}