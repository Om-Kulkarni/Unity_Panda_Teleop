using UnityEngine;

/// <summary>
/// Manages the visibility of different UI panels in the main menu.
/// </summary>
public class MenuManager : MonoBehaviour
{
    [Header("UI Panels")]
    [Tooltip("The initial UI panel that is shown.")]
    public GameObject startPanel;

    [Tooltip("The Database UI Panel.")]
    public GameObject databasePanel;

    [Tooltip("The Color Selection UI Panel.")]
    public GameObject colorSelectionPanel;

    [Tooltip("The Size Selection UI Panel.")]
    public GameObject sizeSelectionPanel;

    // Start is called before the first frame update
    void Start()
    {
        // Ensure we start with the correct UI panel visible.
        if (startPanel != null)
        {
            startPanel.SetActive(true);
        }
        if (databasePanel != null)
        {
            databasePanel.SetActive(false);
        }
        if (colorSelectionPanel != null)
        {
            colorSelectionPanel.SetActive(false);
        }
        if (sizeSelectionPanel != null)
        {
            sizeSelectionPanel.SetActive(false);
        }
    }

    /// <summary>
    /// Deactivates the start panel and activates the database panel.
    /// This method should be called by the 'Confirm' button on the Start UI.
    /// </summary>
    public void ShowDatabasePanel()
    {
        if (startPanel != null)
        {
            startPanel.SetActive(false);
        }
        else
        {
            Debug.LogError("Start Panel is not assigned in the MenuManager.");
        }

        if (databasePanel != null)
        {
            databasePanel.SetActive(true);
        }
        else
        {
            Debug.LogError("Database Panel is not assigned in the MenuManager.");
        }
    }

    /// <summary>
    /// Activates the size selection panel.
    /// This method can be called by a button in the database panel.
    /// </summary>
    public void ShowColorSelectionPanel()
    {
        if (databasePanel != null)
        {
            databasePanel.SetActive(false);
        }
        else
        {
            Debug.LogError("Database Panel is not assigned in the MenuManager.");
        }

        if (colorSelectionPanel != null)
        {
            colorSelectionPanel.SetActive(true);
        }
        else
        {
            Debug.LogError("Color Selection Panel is not assigned in the MenuManager.");
        }
    }

    /// <summary>
    /// Activates the database panel.
    /// This method can be called by a button in the color selection panel.
    /// </summary>
    public void BackToDatabasePanel_Color()
    {
        if (colorSelectionPanel != null)
        {
            colorSelectionPanel.SetActive(false);
        }
        else
        {
            Debug.LogError("Color Selection Panel is not assigned in the MenuManager.");
        }

        if (databasePanel != null)
        {
            databasePanel.SetActive(true);
        }
        else
        {
            Debug.LogError("Database Panel is not assigned in the MenuManager.");
        }
    }

    /// <summary>
    /// Activates the size selection panel.
    /// This method can be called by a button in the database panel.
    /// </summary>
    public void ShowSizeSelectionPanel()
    {
        if (databasePanel != null)
        {
            databasePanel.SetActive(false);
            sizeSelectionPanel.SetActive(true);
        }
        else
        {
            Debug.LogError("Database Panel is not assigned in the MenuManager.");
        }
    }

    /// <summary>
    /// Activates the database panel from the size selection panel.
    /// This method can be called by a button in the size selection panel.
    /// </summary>
    public void BackToDatabasePanel_Size()
    {
        Debug.Log("Back to Database Panel from Size Selection");

        if (sizeSelectionPanel != null)
        {
            sizeSelectionPanel.SetActive(false);
        }
        else
        {
            Debug.LogError("Size Selection Panel is not assigned in the MenuManager.");
        }

        if (databasePanel != null)
        {
            databasePanel.SetActive(true);
        }
        else
        {
            Debug.LogError("Database Panel is not assigned in the MenuManager.");
        }
    }
}
