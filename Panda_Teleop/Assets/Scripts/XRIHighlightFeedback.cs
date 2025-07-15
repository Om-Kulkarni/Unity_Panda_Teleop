using System.Collections;
using System.Collections.Generic;
using System.Drawing.Text;
using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;

/// <summary>
/// This script listens to events from an XRBaseInteractable component (like XRGrabInteractable)
/// and changes the object's color to provide visual feedback for hovering.
/// </summary>
///

[RequireComponent(typeof(XRBaseInteractable))]
[RequireComponent(typeof(Renderer))]

public class XRIHighlightFeedback : MonoBehaviour
{
    [Tooltip("The color to change to when the object is being hovered over.")]
    public Color highlightColor = Color.yellow;

    private XRBaseInteractable interactable;
    private Renderer objectRenderer;
    private Color originalColor;

    private void Awake()
    {
        interactable = GetComponent<XRBaseInteractable>();
        objectRenderer = GetComponent<Renderer>();
        originalColor = objectRenderer.material.color;
    }

    private void OnEnable()
    {
        interactable.hoverEntered.AddListener(OnHoverEntered);
        interactable.hoverExited.AddListener(OnHoverExited);
    }

    private void OnDisable()
    {
        interactable.hoverEntered.RemoveListener(OnHoverEntered);
        interactable.hoverExited.RemoveListener(OnHoverExited);
    }

    private void OnHoverEntered(HoverEnterEventArgs args)
    {
        // Change the object's color to the highlight color when hovered
        objectRenderer.material.color = highlightColor;
    }

    private void OnHoverExited(HoverExitEventArgs args)
    {
        // Revert the object's color back to the original color when hover ends
        objectRenderer.material.color = originalColor;
    }
}
