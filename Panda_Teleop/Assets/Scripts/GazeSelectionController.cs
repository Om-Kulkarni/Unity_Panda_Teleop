using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine.XR.Interaction.Toolkit;
using Unity.Robotics.PickAndPlace;

/// <summary>
/// Manages the pick-and-place selection process using eye-gaze.
/// This script acts as a state machine to guide the user through the selection process.
/// 1. Awaits selection of a source object ("Target" tag).
/// 2. Awaits selection of a destination object ("TargetPlacement" tag).
/// 3. Triggers the PandaTrajectoryPlanner to execute the robot's movement.
/// </summary>
///

public class GazeSelectionController : MonoBehaviour
{
    // --- Public References to be set in the Inspector ---
    [Header("Core Components")]
    [Tooltip("Reference to the CustomGazeInteractor script on your Main Camera.")]
    public CustomGazeInteractor gazeInteractor;
    [Tooltip("Reference to the PandaTrajectoryPlanner script in the scene.")]
    public PandaTrajectoryPlanner pandaTrajectoryPlanner;

    [Header("Input Action")]
    [Tooltip("The Input Action to trigger a selection (e.g., a controller button).")]
    public InputActionReference selectInputAction;

    // --- State Machine Variables ---
    private enum SelectionState { AwaitingSource, AwaitingDestination }
    private SelectionState currentState = SelectionState.AwaitingSource;

    private Transform selectedSource = null;
    private Transform selectedDestination = null;

    private void OnEnable()
    {
        // Enable the input action when this script is enabled
        selectInputAction.action.Enable();
        selectInputAction.action.performed += OnSelectPerformed;
    }

    private void OnDisable()
    {
        // Disable the input action when this script is disabled
        selectInputAction.action.performed -= OnSelectPerformed;
        selectInputAction.action.Disable();
    }

    /// <summary>
    /// This method is called every time the 'selectInputAction' is performed (e.g., button press).
    /// </summary>
    ///
    private void OnSelectPerformed(InputAction.CallbackContext context)
    {
        // Ask the gazeInteractor what object the user is currently looking at.
        GameObject hoveredObject = gazeInteractor.GetHoveredObject();

        // If no object is hovered, do nothing.
        if (hoveredObject == null) { return; }

        // -- State Machine Logic --
        // Check the current state and handle the selection accordingly.
        if (currentState == SelectionState.AwaitingSource)
        {
            // Check if the robot is currently busy executing a trajectory
            if (!CanStartNewSelection())
            {
                Debug.Log("Robot is currently busy. Please wait for the current operation to complete.");
                return;
            }

            // If we are waiting for a source, check if the object has the corect tag.
            if (hoveredObject.CompareTag("Target"))
            {
                // Store the selected source object and update the state.
                selectedSource = hoveredObject.transform;
                currentState = SelectionState.AwaitingDestination;
                // Optionally, you can provide feedback to the user.
                Debug.Log("Source selected: " + selectedSource.name);
            }
        }
        else if (currentState == SelectionState.AwaitingDestination)
        {
            // If we are waiting for a destination, check if the object has the correct tag.
            if (hoveredObject.CompareTag("TargetPlacement"))
            {
                // Check if the robot is currently busy executing a trajectory
                if (!CanStartNewSelection())
                {
                    Debug.Log("Robot is currently busy. Please wait for the current operation to complete.");
                    return;
                }

                // Store the selected destination object and update the state.
                selectedDestination = hoveredObject.transform;
                Debug.Log("Destination selected: " + selectedDestination.name);

                // Assign our selected objects to the public properties of the Panda trajectory planner script
                pandaTrajectoryPlanner.Target = selectedSource.gameObject;
                pandaTrajectoryPlanner.TargetPlacement = selectedDestination.gameObject;

                // Call the method to execute the pick and place operation
                pandaTrajectoryPlanner.ExecutePickAndPlace();

                // Reset the state machine for the next selection.
                ResetSelection();
            }
        }
    }

    /// <summary>
    /// Resets the state machine back to its initial state.
    /// </summary>
    ///
    private void ResetSelection()
    {
        currentState = SelectionState.AwaitingSource;
        selectedSource = null;
        selectedDestination = null;
        // Optionally, you can provide feedback to the user.
        Debug.Log("Selection reset. Awaiting new source selection.");
    }

    /// <summary>
    /// Check if we can start a new selection (robot is not busy)
    /// </summary>
    private bool CanStartNewSelection()
    {
        if (pandaTrajectoryPlanner.IsExecutingTrajectory)
        {
            // Could add UI feedback here in the future
            return false;
        }
        return true;
    }
}
