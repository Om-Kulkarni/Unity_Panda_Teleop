using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class InitializeFrankaPose : MonoBehaviour
{
    [Header("Franka Arm Configuration")]
    [Tooltip("This script should be attached directly to the Franka arm GameObject")]
    [HideInInspector]
    public GameObject frankaArm;
    
    [Header("Initial Joint Angles (in degrees)")]
    [Tooltip("Joint angles for a natural pose with end effector in front of user")]
    public float[] initialJointAngles = new float[]
    {
        0f,    // panda_joint1 (shoulder pan)
        -45f,  // panda_joint2 (shoulder lift) - lowered
        0f,    // panda_joint3 (upper arm roll)
        -135f, // panda_joint4 (elbow flex) - bent to bring arm forward
        0f,    // panda_joint5 (forearm roll)
        90f,   // panda_joint6 (wrist flex) - angled up
        45f    // panda_joint7 (wrist roll) - slight rotation
    };
    
    [Header("Settings")]
    [Tooltip("Apply pose automatically on Start")]
    public bool applyOnStart = true;
    
    [Tooltip("Delay before applying pose (seconds)")]
    public float initializationDelay = 0.5f;

    private ArticulationBody[] articulationChain;

    void Start()
    {
        if (applyOnStart)
        {
            StartCoroutine(InitializePoseWithDelay());
        }
    }

    private IEnumerator InitializePoseWithDelay()
    {
        yield return new WaitForSeconds(initializationDelay);
        SetFrankaPose();
    }

    [ContextMenu("Set Franka Pose")]
    public void SetFrankaPose()
    {
        // Use this GameObject since script is attached to the Franka arm
        frankaArm = this.gameObject;

        // Get all articulation bodies in the chain
        articulationChain = frankaArm.GetComponentsInChildren<ArticulationBody>();
        
        if (articulationChain.Length == 0)
        {
            Debug.LogError("InitializeFrankaPose: No ArticulationBody components found in Franka arm!");
            return;
        }

        Debug.Log($"InitializeFrankaPose: Found {articulationChain.Length} articulation bodies");

        // Apply joint angles
        int jointIndex = 0;
        foreach (ArticulationBody joint in articulationChain)
        {
            // Skip the base link (usually immovable)
            if (joint.isRoot || joint.jointType == ArticulationJointType.FixedJoint)
            {
                continue;
            }

            if (jointIndex < initialJointAngles.Length)
            {
                SetJointAngle(joint, initialJointAngles[jointIndex]);
                jointIndex++;
            }
        }

        Debug.Log("InitializeFrankaPose: Franka arm pose initialized successfully!");
    }

    // Method to reset to zero pose
    [ContextMenu("Reset to Zero Pose")]
    public void ResetToZeroPose()
    {
        if (articulationChain == null)
        {
            articulationChain = this.gameObject.GetComponentsInChildren<ArticulationBody>();
        }

        if (articulationChain != null)
        {
            foreach (ArticulationBody joint in articulationChain)
            {
                if (!joint.isRoot && joint.jointType != ArticulationJointType.FixedJoint)
                {
                    SetJointAngle(joint, 0f);
                }
            }
        }
    }

    private void SetJointAngle(ArticulationBody joint, float angleDegrees)
    {
        if (joint.jointType == ArticulationJointType.RevoluteJoint)
        {
            var drive = joint.xDrive;
            drive.target = angleDegrees;
            joint.xDrive = drive;
        }
        else if (joint.jointType == ArticulationJointType.PrismaticJoint)
        {
            var drive = joint.xDrive;
            drive.target = angleDegrees * 0.01f; // Convert to meters if needed
            joint.xDrive = drive;
        }
    }
}
