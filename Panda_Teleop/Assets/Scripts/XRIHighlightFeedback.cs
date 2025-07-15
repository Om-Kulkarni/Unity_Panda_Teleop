using System.Collections;
using System.Collections.Generic;
using System.Drawing.Text;
using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;

/// <summary>
/// This script listens to events from an XRBaseInteractable component (like XRGrabInteractable)
/// and changes the object's color to provide visual feedback for hovering and grabbing.
/// </summary>
///

[RequireComponent(typeof(XRBaseInteractable))]
[RequireComponent(typeof(Renderer))]

public class XRIHighlightFeedback : MonoBehaviour
{
    [Tooltip("The color to change to when the object is being hovered over.")]
    public Color hoverColor = Color.yellow;
    
    [Tooltip("The color to change to when the object is being grabbed.")]
    public Color grabbedColor = Color.green;

    private XRBaseInteractable interactable;
    private Renderer objectRenderer;
    private Color originalColor;
    private bool isGrabbed = false;

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
        interactable.selectEntered.AddListener(OnGrabStarted);
        interactable.selectExited.AddListener(OnGrabEnded);
    }

    private void OnDisable()
    {
        interactable.hoverEntered.RemoveListener(OnHoverEntered);
        interactable.hoverExited.RemoveListener(OnHoverExited);
        interactable.selectEntered.RemoveListener(OnGrabStarted);
        interactable.selectExited.RemoveListener(OnGrabEnded);
    }

    private void OnHoverEntered(HoverEnterEventArgs args)
    {
        // Only change to hover color if not already grabbed
        if (!isGrabbed)
        {
            objectRenderer.material.color = hoverColor;
        }
    }

    private void OnHoverExited(HoverExitEventArgs args)
    {
        // Only revert to original color if not grabbed
        if (!isGrabbed)
        {
            objectRenderer.material.color = originalColor;
        }
    }
    
    private void OnGrabStarted(SelectEnterEventArgs args)
    {
        isGrabbed = true;
        // Grabbed color takes priority over hover color
        objectRenderer.material.color = grabbedColor;
    }
    
    private void OnGrabEnded(SelectExitEventArgs args)
    {
        isGrabbed = false;
        // Check if still hovering after grab ends
        if (interactable.isHovered)
        {
            objectRenderer.material.color = hoverColor;
        }
        else
        {
            objectRenderer.material.color = originalColor;
        }
    }
}
