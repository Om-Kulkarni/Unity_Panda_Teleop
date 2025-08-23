using UnityEngine;
using UnityEngine.InputSystem;
using Unity.Robotics.PickAndPlace;

public class GazeSelectionController : MonoBehaviour
{
    [Header("Core Components")]
    [Tooltip("Reference to the CustomGazeInteractor script on your Main Camera.")]
    public CustomGazeInteractor gazeInteractor;
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

    //-- NEW --//
    // Flags to ensure each error only happens once per session.
    private bool hasLightPurpleErrorOccurred = false;

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
        GameObject hoveredObject = gazeInteractor.GetHoveredObject();
        if (hoveredObject == null) { return; }

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

                    //-- MODIFIED --//

                    // ERROR 1: If user selects "light purple" for the FIRST TIME, swap it for "pink".
                    if (materialName.Contains("Light Purple") && !hasLightPurpleErrorOccurred)
                    {
                        Debug.LogWarning("FIRST TIME ERROR TRIGGERED: Light purple cube selected. Swapping to pink cube.");
                        HandleLightPurpleError(hoveredObject.transform);
                        return; // Exit after handling the error
                    }
                }
                //-- ERROR INJECTION END --//

                // This is the original, correct behavior
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

                currentState = SelectionState.AwaitingSource;
                selectedSource = null;
                selectedDestination = null;
            }

            else if (hoveredObject.CompareTag("Target"))
            {
                Debug.Log("New source selected, resetting previous choice.");
                ResetSelection();
                OnSelectPerformed(context);
                return;
            }
        }
    }


    //-- ERROR INJECTION START --//

    private void HandleLightPurpleError(Transform sourceSelection)
    {
        GameObject destinationOverride = GameObject.Find("TargetPlacement (6)");

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
            currentState = SelectionState.AwaitingSource;
            selectedSource = null;
            selectedDestination = null;
            //-- NEW --//
            // Set the flag so this error doesn't happen again.
            hasLightPurpleErrorOccurred = true;
        }
        else
        {
            Debug.LogError("ERROR FAILED: Could not find a GameObject named 'PlacementArea_7' in the scene.");
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

        if (activeSourceGlower != null)
        {
            activeSourceGlower.SetGlow(false);
            activeSourceGlower = null;
        }
        Debug.Log("Selection reset. Awaiting new source selection.");
    }

    private bool CanStartNewSelection()
    {
        return !pandaTrajectoryPlanner.IsExecutingTrajectory;
    }
}