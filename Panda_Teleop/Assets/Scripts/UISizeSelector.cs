using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class UISizeSelector : MonoBehaviour
{
    [Header("UI Elements")]
    [Tooltip("The slider that controls the size of the preview cube.")]
    public Slider sizeSlider;

    [Header("Preview Object")]
    [Tooltip("The prefab of the cube to be spawned for preview.")]
    public GameObject previewCubePrefab;

    [Tooltip("The position where the preview cube will be spawned, relative to this UI panel.")]
    public Vector3 previewSpawnOffset = new Vector3(1.5f, 0, 0);

    [Tooltip("The existing TextMeshPro object in the scene to use for the preview.")]
    public TextMeshProUGUI previewTMPInstance;

    [Tooltip("The TextMeshPro object in Database Panel.")]
    public TextMeshProUGUI databaseTMPInstance;

    private GameObject previewInstance;

    void OnEnable()
    {
        // Instantiate the preview cube when the UI is enabled.
        if (previewCubePrefab != null)
        {
            // 1. Instantiate the prefab.
            previewInstance = Instantiate(previewCubePrefab);
            
            // 2. Parent the new cube to this UI panel's transform.
            previewInstance.transform.SetParent(transform, false);

            // 3. Set its position and rotation using the local coordinate system.
            previewInstance.transform.localPosition = previewSpawnOffset;
            previewInstance.transform.localRotation = Quaternion.identity;

            // 4. Set initial size from slider.
            OnSliderValueChanged(sizeSlider.value);
        }
        else
        {
            Debug.LogError("Preview Cube Prefab is not assigned in the UISizeSelector.");
        }

        // Add a listener to the slider to catch value changes.
        if (sizeSlider != null)
        {
            sizeSlider.onValueChanged.AddListener(OnSliderValueChanged);
        }
    }

    void OnDisable()
    {
        // Destroy the preview cube when the UI is disabled.
        if (previewInstance != null)
        {
            Destroy(previewInstance);
        }

        // It's good practice to remove listeners when the object is disabled.
        if (sizeSlider != null)
        {
            sizeSlider.onValueChanged.RemoveListener(OnSliderValueChanged);
        }
    }

    /// <summary>
    /// Called when the slider's value changes. Updates the preview cube's scale.
    /// </summary>
    /// <param name="value">The current value of the slider.</param>
    public void OnSliderValueChanged(float value)
    {
        if (previewInstance != null)
        {
            // Update the scale of the cube based on the slider's value.
            previewInstance.transform.localScale = Vector3.one * value;

            // Update the size value text to reflect the current size.
            if (previewTMPInstance != null && databaseTMPInstance != null)
            {
                previewTMPInstance.text = value.ToString("F2");
                databaseTMPInstance.text = value.ToString("F2");
                Debug.Log("Size Value Text updated to: " + value.ToString("F2"));
            }
            else
            {
                Debug.LogError("Size Value Text is not assigned in the UISizeSelector.");
            }
        }
    }    
    
    /// <summary>
    /// This method can be called by a "Confirm" button.
    /// It finalizes the selection and deactivates this panel.
    /// </summary>
    public void ConfirmSelection()
    {
        // Here you would typically pass the selected size to another manager.
        // For example: GameManager.Instance.SetObjectSize(sizeSlider.value);
        Debug.Log("Confirmed size: " + sizeSlider.value);

        // Deactivate the size selector panel.
        gameObject.SetActive(false);
    }
    
}
