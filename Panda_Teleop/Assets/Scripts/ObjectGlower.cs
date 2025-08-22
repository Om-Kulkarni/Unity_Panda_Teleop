using UnityEngine;

/// <summary>
/// A simple component that controls the emission glow of an object's material.
/// It waits for external commands to turn the glow on or off.
/// </summary>
public class ObjectGlower : MonoBehaviour
{
    [Tooltip("The emission color to apply when the glow is active.")]
    public Color glowColor = new Color(0.4f, 0.4f, 0.4f);

    private Material instancedMaterial;
    private Color originalEmissionColor;
    private static readonly int EmissionColorID = Shader.PropertyToID("_EmissionColor");

    void Awake()
    {
        // Create a unique material instance for this object to prevent affecting others.
        instancedMaterial = GetComponent<Renderer>().material;
        instancedMaterial.EnableKeyword("_EMISSION");
        originalEmissionColor = instancedMaterial.GetColor(EmissionColorID);
    }

    /// <summary>
    /// Turns the emission glow on.
    /// </summary>
    public void SetGlow(bool isGlowing)
    {
        if (instancedMaterial == null) return;
        
        instancedMaterial.SetColor(EmissionColorID, isGlowing ? glowColor : originalEmissionColor);
    }
}