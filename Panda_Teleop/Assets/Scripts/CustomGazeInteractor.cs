using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using VIVE.OpenXR;
using VIVE.OpenXR.EyeTracker;
using UnityEngine.XR.Interaction.Toolkit;

/// <summary>
/// This script calculates a stable "Cyclopean" gaze vector, smooths it,
/// and uses it to position a visualizer and trigger hover events on XRI Interactables.
/// </summary>
/// 

[RequireComponent(typeof(LineRenderer))] // We now require a LineRenderer component
public class CustomGazeInteractor : MonoBehaviour
{
    [Header("Visualization")]
    [Tooltip("Assign a GameObject here (like a small blue sphere) to visualize the gaze point.")]
    public Transform gazeVisualizer; // The cursor object to move

    [Header("Raycast Properties")]
    [Tooltip("The maximum distance to cast the gaze ray.")]
    public float maxRayDistance = 10f;
    [Tooltip("Layer mask to define which objects are interactable.")]
    public LayerMask interactionLayerMask;
    [Tooltip("Default distance to place the visualizer when not hitting anything.")]
    public float defaultVisualizerDistance = 0.5f;

    [Header("Smoothing")]
    [Tooltip("How quickly the gaze point catches up. Lower values are smoother.")]
    [Range(0.1f, 0.3f)]
    public float smoothingFactor = 0.2f;

    // To keep track of the object we are currently looking at.
    private IXRHoverInteractable currentHoveredInteractable = null;
    private Vector3 smoothedGazeDirection;
    private LineRenderer gazeRayLine; // Reference to our LineRenderer

    public GameObject GetHoveredObject()
    {
        // Returns the currently hovered object, if any.
        if (currentHoveredInteractable != null)
        {
            return (currentHoveredInteractable as MonoBehaviour).gameObject;
        }
        return null;
    }

    // Start is called before the first frame update
    void Start()
    {
        // Get the LineRenderer component on this same GameObject.
        gazeRayLine = GetComponent<LineRenderer>();

        // Initialize with the camera's transform as a fallback.
        smoothedGazeDirection = transform.forward;

        // Hide the visualizer initially if it exists
        if (gazeVisualizer != null) { gazeVisualizer.gameObject.SetActive(false); }
    }

    // Update is called once per frame
    void Update()
    {
        Vector3 combinedGazeDirection = transform.forward; // Default to head-gaze direction

        // --- Step 1: Get Combined Eye Data ---
        XrSingleEyeGazeDataHTC[] out_gazes;
        if (XR_HTC_eye_tracker.Interop.GetEyeGazeData(out out_gazes))
        {
            // Get data for both left and right eyes
            var leftEye = out_gazes[(int)XrEyePositionHTC.XR_EYE_POSITION_LEFT_HTC];
            var rightEye = out_gazes[(int)XrEyePositionHTC.XR_EYE_POSITION_RIGHT_HTC];

            // Proceed only if both eyes have valid data
            if (leftEye.isValid && rightEye.isValid)
            {
                // --- CYCLOPEAN GAZE CALCULATION ---
                Vector3 leftGazeDirection = leftEye.gazePose.orientation.ToUnityQuaternion() * Vector3.forward;
                Vector3 rightGazeDirection = rightEye.gazePose.orientation.ToUnityQuaternion() * Vector3.forward;

                // The final gaze direction is the average of both eyes' gaze directions
                combinedGazeDirection = (leftGazeDirection + rightGazeDirection).normalized;
            }
        }

        // -- Step 2: Smooth the Gaze Direction ---
        // We spherical-lerp from the last smoothed direction to the new combined gaze direction
        smoothedGazeDirection = Vector3.Slerp(smoothedGazeDirection, combinedGazeDirection, smoothingFactor);

        // Ensure visualizer is active only when we have valid tracking
        if (gazeVisualizer != null && !gazeVisualizer.gameObject.activeInHierarchy)
        {
            gazeVisualizer.gameObject.SetActive(true);
        }

        // --- Step 3: Perform the Raycast ---
        RaycastHit hit;
        Vector3 rayStartPoint = transform.position;
        if (Physics.Raycast(rayStartPoint, smoothedGazeDirection, out hit, maxRayDistance, interactionLayerMask))
        {
            UpdateVisuals(rayStartPoint, hit.point, hit.normal);
            IXRHoverInteractable hitInteractable = hit.collider.GetComponent<IXRHoverInteractable>();
            HandleHover(hitInteractable);
        }
        else
        {
            Vector3 endPoint = rayStartPoint + (smoothedGazeDirection * defaultVisualizerDistance);
            UpdateVisuals(rayStartPoint, endPoint, -smoothedGazeDirection);
            ClearHover();
        }
    }

    private void UpdateVisuals(Vector3 startPoint, Vector3 endPoint, Vector3 normal)
    {
        // Update the Line Renderer
        if (gazeRayLine != null)
        {
            gazeRayLine.SetPosition(0, startPoint);
            gazeRayLine.SetPosition(1, endPoint);
        }

        // Update the Gaze Visualizer Sphere
        if (gazeVisualizer != null)
        {
            gazeVisualizer.position = endPoint;
            gazeVisualizer.rotation = Quaternion.LookRotation(normal);
        }
    }

    private void HandleHover(IXRHoverInteractable hitInteractable)
    {
        if (hitInteractable != null)
        {
            if (hitInteractable != currentHoveredInteractable)
            {
                if (currentHoveredInteractable != null)
                {
                    currentHoveredInteractable.OnHoverExited(new HoverExitEventArgs { interactorObject = null, interactableObject = currentHoveredInteractable });
                }
                hitInteractable.OnHoverEntered(new HoverEnterEventArgs { interactorObject = null, interactableObject = hitInteractable });
                currentHoveredInteractable = hitInteractable;
            }
        }
        else
        {
            ClearHover();
        }
    }

    private void ClearHover()
    {
        if (currentHoveredInteractable != null)
        {
            currentHoveredInteractable.OnHoverExited(new HoverExitEventArgs { interactorObject = null, interactableObject = currentHoveredInteractable });
            currentHoveredInteractable = null;
        }
    }
}
