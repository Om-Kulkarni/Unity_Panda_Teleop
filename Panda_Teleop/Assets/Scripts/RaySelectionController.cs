using UnityEngine;
using UnityEngine.InputSystem;
using Unity.Robotics.PickAndPlace;
using UnityEngine.XR.Interaction.Toolkit; // Required for Ray Interactor
using System.Linq; // Required for .FirstOrDefault()

public class RaySelectionController : MonoBehaviour
{
    [Header("Core Components")]
    [Tooltip("Reference to the XRRayInteractor on your controller.")]
    public XRRayInteractor rayInteractor;
    
    [Tooltip("Reference to the PandaTrajectoryPlanner script in the scene.")]
    public PandaTrajectoryPlanner pandaTrajectoryPlanner;

    [Header("Input Action")]
    [Tooltip("The Input Action to trigger a selection (e.g., a controller button).")]
    public InputActionReference selectInputAction;

    private enum SelectionState { AwaitingSource, AwaitingDestination }
    private SelectionState currentState = SelectionState.AwaitingSource;

    private Transform selectedSource = null;
    private Transform selectedDestination = null;
    
    private ObjectGlower activeSourceGlower = null;
    private ObjectGlower activeDestinationGlower = null;

    // Flags to ensure each error only happens once per session.
    private bool hasLightPurpleErrorOccurred = false;
    private bool hasLightGreenErrorOccurred = false;

    private void OnEnable()
    {
        selectInputAction.action.Enable();
        selectInputAction.action.performed += OnSelectPerformed;
    }

    private void OnDisable()
    {
        selectInputAction.action.performed -= OnSelectPerformed;
        selectInputAction.action.Disable();
    }

    private void OnSelectPerformed(InputAction.CallbackContext context)
    {
        // Check if the ray interactor is valid and is currently hovering over an object.
        if (rayInteractor == null || rayInteractor.interactablesHovered.Count == 0)
        {
            return; // Exit if not pointing at anything.
        }

        // Get the GameObject the ray is pointing at.
        var firstHoveredInteractable = rayInteractor.interactablesHovered.FirstOrDefault();
        if (firstHoveredInteractable == null) return;
        
        GameObject hoveredObject = (firstHoveredInteractable as MonoBehaviour)?.gameObject;
        if (hoveredObject == null) return;

        // The rest of the logic is the same as before.
        if (currentState == SelectionState.AwaitingSource)
        {
            if (!CanStartNewSelection())
            {
                Debug.Log("Robot is currently busy. Please wait.");
                return;
            }
            
            ClearPreviousSelectionGlows();

            if (hoveredObject.CompareTag("Target"))
            {
                //-- ERROR INJECTION START --//
                Renderer selectedRenderer = hoveredObject.GetComponent<Renderer>();
                if (selectedRenderer != null)
                {
                    string materialName = selectedRenderer.material.name;

                    // ERROR 1: If user selects "light purple" for the FIRST TIME, swap it for "pink".
                    if (materialName.Contains("Light Purple") && !hasLightPurpleErrorOccurred)
                    {
                        Debug.LogWarning("FIRST TIME ERROR TRIGGERED: Light purple cube selected. Swapping to pink cube.");
                        HandleLightPurpleError();
                        return;
                    }

                    // ERROR 2: If user selects "Light Green" for the FIRST TIME, force destination.
                    if (materialName.Contains("Light Green") && !hasLightGreenErrorOccurred)
                    {
                        Debug.LogWarning("FIRST TIME ERROR TRIGGERED: Light green cube selected. Overriding destination.");
                        HandleLightGreenError(hoveredObject.transform);
                        return;
                    }
                }
                //-- ERROR INJECTION END --//
                
                // Normal behavior
                selectedSource = hoveredObject.transform;
                currentState = SelectionState.AwaitingDestination;
                Debug.Log("Source selected: " + selectedSource.name);
                
                activeSourceGlower = selectedSource.GetComponent<ObjectGlower>();
                if (activeSourceGlower != null)
                {
                    activeSourceGlower.SetGlow(true);
                }
            }
        }
        else if (currentState == SelectionState.AwaitingDestination)
        {
            if (hoveredObject.CompareTag("TargetPlacement"))
            {
                if (!CanStartNewSelection())
                {
                    Debug.Log("Robot is currently busy. Please wait.");
                    return;
                }

                selectedDestination = hoveredObject.transform;
                Debug.Log("Destination selected: " + selectedDestination.name);
                
                activeDestinationGlower = selectedDestination.GetComponent<ObjectGlower>();
                if (activeDestinationGlower != null)
                {
                    activeDestinationGlower.SetGlow(true);
                }

                pandaTrajectoryPlanner.Target = selectedSource.gameObject;
                pandaTrajectoryPlanner.TargetPlacement = selectedDestination.gameObject;
                pandaTrajectoryPlanner.ExecutePickAndPlace();
                
                ResetToInitialState();
            }
            else if (hoveredObject.CompareTag("Target"))
            {
                Debug.Log("New source selected, resetting previous choice.");
                ResetSelection();
                OnSelectPerformed(context); // Re-run the selection logic for the new target
            }
        }
    }
    
    #region Error Handling and State Management
    //-- ERROR INJECTION START --//
    private void HandleLightPurpleError()
    {
        GameObject pinkCube = GameObject.FindGameObjectsWithTag("Target")
            .FirstOrDefault(t => t.GetComponent<Renderer>()?.material.name.Contains("Pink") ?? false);

        if (pinkCube != null)
        {
            selectedSource = pinkCube.transform;
            currentState = SelectionState.AwaitingDestination;
            Debug.Log("ERROR APPLIED: Source selection swapped to: " + selectedSource.name);
            
            activeSourceGlower = selectedSource.GetComponent<ObjectGlower>();
            if (activeSourceGlower != null)
            {
                activeSourceGlower.SetGlow(true);
            }
            
            hasLightPurpleErrorOccurred = true;
        }
        else
        {
            Debug.LogError("ERROR FAILED: Could not find a 'pink' cube in the scene to swap to.");
            ResetSelection();
        }
    }
    
    private void HandleLightGreenError(Transform sourceSelection)
    {
        GameObject destinationOverride = GameObject.Find("TargetPlacement (7)");

        if (destinationOverride != null)
        {
            selectedSource = sourceSelection;
            selectedDestination = destinationOverride.transform;

            Debug.Log("ERROR APPLIED: Destination for '" + selectedSource.name + "' has been forced to '" + selectedDestination.name + "'.");

            activeSourceGlower = selectedSource.GetComponent<ObjectGlower>();
            if (activeSourceGlower != null) activeSourceGlower.SetGlow(true);
            
            activeDestinationGlower = selectedDestination.GetComponent<ObjectGlower>();
            if (activeDestinationGlower != null) activeDestinationGlower.SetGlow(true);

            pandaTrajectoryPlanner.Target = selectedSource.gameObject;
            pandaTrajectoryPlanner.TargetPlacement = selectedDestination.gameObject;
            pandaTrajectoryPlanner.ExecutePickAndPlace();

            hasLightGreenErrorOccurred = true;
            ResetToInitialState();
        }
        else
        {
            Debug.LogError("ERROR FAILED: Could not find 'TargetPlacement (7)' in the scene.");
            ResetSelection();
        }
    }
    //-- ERROR INJECTION END --//
    
    private void ClearPreviousSelectionGlows()
    {
        if (activeSourceGlower != null)
        {
            activeSourceGlower.SetGlow(false);
            activeSourceGlower = null;
        }

        if (activeDestinationGlower != null)
        {
            activeDestinationGlower.SetGlow(false);
            activeDestinationGlower = null;
        }
    }
    
    private void ResetSelection()
    {
        currentState = SelectionState.AwaitingSource;
        selectedSource = null;
        selectedDestination = null;
        ClearPreviousSelectionGlows();
        Debug.Log("Selection reset. Awaiting new source selection.");
    }

    private void ResetToInitialState()
    {
        currentState = SelectionState.AwaitingSource;
        selectedSource = null;
        selectedDestination = null;
        // The glows will be cleared on the next selection or after the robot finishes.
    }

    private bool CanStartNewSelection()
    {
        return !pandaTrajectoryPlanner.IsExecutingTrajectory;
    }
    #endregion
}