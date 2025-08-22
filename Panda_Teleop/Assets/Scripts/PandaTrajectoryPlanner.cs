using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter;
using RosMessageTypes.PandaMoveit;
using RosMessageTypes.Geometry;
using RosMessageTypes.Trajectory;
using RosMessageTypes.Moveit;

namespace Unity.Robotics.PickAndPlace
{
    /// <summary>
    /// Trajectory planner for the Franka Panda robot using MoveIt.
    /// This version includes a final upward lift movement after placing an object.
    /// </summary>
    public class PandaTrajectoryPlanner : MonoBehaviour
    {
        [Header("System References")]
        [Tooltip("The DatabaseManager script in the scene.")]
        [SerializeField]
        private DatabaseManager m_DatabaseManager;

        const int k_NumRobotJoints = 7;
        const string k_RosServiceName = "panda_moveit";

        public static readonly string[] LinkNames =
        {
            "world/panda_link0/panda_link1", "/panda_link2", "/panda_link3", "/panda_link4",
            "/panda_link5", "/panda_link6", "/panda_link7"
        };

        [Header("ROS and Robot Setup")]
        [SerializeField]
        string m_RosServiceName = "panda_moveit";
        public string RosServiceName { get => m_RosServiceName; set => m_RosServiceName = value; }

        [SerializeField]
        [Tooltip("The GameObject representing the Panda robot")]
        GameObject m_PandaRobot;
        public GameObject PandaRobot { get => m_PandaRobot; set => m_PandaRobot = value; }

        [SerializeField]
        GameObject m_Target;
        public GameObject Target { get => m_Target; set => m_Target = value; }

        [SerializeField]
        GameObject m_TargetPlacement;
        public GameObject TargetPlacement { get => m_TargetPlacement; set => m_TargetPlacement = value; }
        
        [SerializeField]
        [Tooltip("MoveIt planning group to use for trajectory planning")]
        string m_PlanningGroup = "panda_manipulator";
        public string PlanningGroup => m_PlanningGroup;

        [Header("Movement Parameters")]
        [SerializeField]
        [Tooltip("Speed factor for trajectory execution (0.1 = 10% speed)")]
        float m_SpeedFactor = 0.5f;

        [SerializeField]
        [Tooltip("Height above target for approach (in meters)")]
        float m_PickOffsetHeight = 0.15f;

        [SerializeField]
        [Tooltip("Additional offset for gripper TCP to prevent collision with table (in meters)")]
        float m_GripperTcpOffset = 0.05f;
        
        //-- NEW --//
        [SerializeField]
        [Tooltip("Height to lift the end-effector after placing the object (in meters)")]
        float m_PostPlaceLiftHeight = 0.1f;

        [Header("Gripper Settings")]
        [SerializeField]
        [Tooltip("Gripper open position (meters)")]
        float m_GripperOpenPosition = 0.04f;

        [SerializeField]
        [Tooltip("Gripper closed position (meters)")]
        float m_GripperClosedPosition = 0.0f;

        // Calculated pick pose offset combining approach height and TCP clearance
        Vector3 m_PickPoseOffset => Vector3.up * (m_PickOffsetHeight + m_GripperTcpOffset);
        
        // Base gripper orientation for approaching from above
        readonly Quaternion m_BaseGripperOrientation = Quaternion.Euler(180, 0, 0);

        // Robot joints
        UrdfJointRevolute[] m_JointArticulationBodies;
        
        // Gripper ArticulationBodies
        ArticulationBody m_LeftGripper;
        ArticulationBody m_RightGripper;
        
        // ROS connection
        ROSConnection m_Ros;

        // State tracking
        bool m_IsExecutingTrajectory = false;
        bool m_IsPickAndPlaceComplete = false;

        public bool IsExecutingTrajectory => m_IsExecutingTrajectory;
        public bool IsPickAndPlaceComplete => m_IsPickAndPlaceComplete;

        void Start()
        {
            // Get ROS connection
            m_Ros = ROSConnection.GetOrCreateInstance();
            m_Ros.RegisterRosService<PandaMoverServiceRequest, PandaMoverServiceResponse>(k_RosServiceName);

            // Check if robot is assigned
            if (m_PandaRobot == null)
            {
                Debug.LogError("PandaRobot GameObject is not assigned! Please assign it in the Inspector.");
                return;
            }

            // Initialize joint references
            m_JointArticulationBodies = new UrdfJointRevolute[k_NumRobotJoints];
            var linkName = string.Empty;
            for (var i = 0; i < k_NumRobotJoints; i++)
            {
                linkName += LinkNames[i];
                m_JointArticulationBodies[i] = m_PandaRobot.transform.Find(linkName).GetComponent<UrdfJointRevolute>();
            }

            // Find Panda gripper fingers
            var leftGripperPath = linkName + "/panda_link8/panda_hand/panda_leftfinger";
            var rightGripperPath = linkName + "/panda_link8/panda_hand/panda_rightfinger";
            
            var leftGripperTransform = m_PandaRobot.transform.Find(leftGripperPath);
            var rightGripperTransform = m_PandaRobot.transform.Find(rightGripperPath);
            
            if (leftGripperTransform != null)
                m_LeftGripper = leftGripperTransform.GetComponent<ArticulationBody>();
            if (rightGripperTransform != null)
                m_RightGripper = rightGripperTransform.GetComponent<ArticulationBody>();

            // Initialize gripper to open position
            MoveGripper(m_GripperOpenPosition, m_GripperOpenPosition);

            Debug.Log("PandaTrajectoryPlanner initialized successfully");
        }

        /// <summary>
        /// Execute a full pick and place operation
        /// </summary>
        public void ExecutePickAndPlace()
        {
            if (m_IsExecutingTrajectory)
            {
                Debug.LogWarning("Already executing trajectory, please wait");
                return;
            }

            if (m_Target == null || m_TargetPlacement == null)
            {
                Debug.LogError("Pick and place targets must be set");
                return;
            }

            StartCoroutine(ExecutePickAndPlaceCoroutine());
        }

        /// <summary>
        /// Execute pick and place operation as a coroutine
        /// </summary>
        IEnumerator ExecutePickAndPlaceCoroutine()
        {
            m_IsExecutingTrajectory = true;
            m_IsPickAndPlaceComplete = false;

            // Ensure gripper is open before starting
            yield return StartCoroutine(HandleGripperAction(false));

            // Create service request
            var request = new PandaMoverServiceRequest();
            request.joints_input = GetCurrentJointState();

            // -- Pick Pose ---
            var pickPositionWorld = m_Target.transform.position + m_PickPoseOffset;
            var pickPositionLocal = m_PandaRobot.transform.InverseTransformPoint(pickPositionWorld);
            var pickOrientationWorld = GetPickOrientation(m_Target.transform);
            var pickOrientationLocal = Quaternion.Inverse(m_PandaRobot.transform.rotation) * pickOrientationWorld;
            request.pick_pose = new PoseMsg
            {
                position = pickPositionLocal.To<FLU>(),
                orientation = pickOrientationLocal.To<FLU>()
            };

            // -- Place Pose ---
            var placePositionWorld = m_TargetPlacement.transform.position + m_PickPoseOffset;
            var placePositionLocal = m_PandaRobot.transform.InverseTransformPoint(placePositionWorld);
            request.place_pose = new PoseMsg
            {
                position = placePositionLocal.To<FLU>(),
                orientation = pickOrientationLocal.To<FLU>()
            };

            // Send service request
            bool serviceCallComplete = false;
            PandaMoverServiceResponse response = null;

            m_Ros.SendServiceMessage<PandaMoverServiceResponse>(k_RosServiceName, request, (r) =>
            {
                response = r;
                serviceCallComplete = true;
            });

            // Wait for service response
            yield return new WaitUntil(() => serviceCallComplete);

            if (response == null || response.trajectories.Length == 0)
            {
                Debug.LogError("No trajectory returned from MoverService.");
                m_IsExecutingTrajectory = false;
                yield break;
            }

            // Execute trajectories
            yield return StartCoroutine(ExecuteTrajectories(response));

            m_IsExecutingTrajectory = false;
            m_IsPickAndPlaceComplete = true;
        }

        /// <summary>
        /// Execute the trajectories returned by MoveIt, including the new post-place lift.
        /// </summary>
        IEnumerator ExecuteTrajectories(PandaMoverServiceResponse response)
        {
            if (response.trajectories != null)
            {
                // For every trajectory plan returned
                for (var trajectoryIndex = 0; trajectoryIndex < response.trajectories.Length; trajectoryIndex++)
                {
                    var trajectory = response.trajectories[trajectoryIndex];
                    
                    // Execute the trajectory
                    yield return StartCoroutine(ExecuteTrajectory(trajectory));
                    
                    // Close the gripper if completed executing the trajectory for the Grasp pose
                    if (trajectoryIndex == (int)Poses.Grasp)
                    {
                        if (m_DatabaseManager != null && m_Target != null)
                        {
                            m_DatabaseManager.StartNewDataEntry(m_Target);
                        }
                        yield return StartCoroutine(HandleGripperAction(true));
                    }
                    
                    // Wait for the robot to achieve the final pose from joint assignment
                    yield return new WaitForSeconds(0.5f / m_SpeedFactor);
                }

                // All pick-and-place trajectories have been executed, open the gripper to place the target cube
                yield return StartCoroutine(HandleGripperAction(false));

                // -- MODIFIED: Plan and execute a final upward lift movement --
                Debug.Log("Planning post-place lift trajectory.");

                // Define the target lift pose in world coordinates
                var liftPositionWorld = m_TargetPlacement.transform.position + m_PickPoseOffset + (Vector3.up * m_PostPlaceLiftHeight);
                var liftOrientationWorld = GetPickOrientation(m_Target.transform); // Use same orientation

                // Create a new service request for this single move
                var liftRequest = new PandaMoverServiceRequest
                {
                    joints_input = GetCurrentJointState() // Plan from the current robot state
                };

                // Transform the world pose into the robot's local coordinate frame
                var liftPositionLocal = m_PandaRobot.transform.InverseTransformPoint(liftPositionWorld);
                var liftOrientationLocal = Quaternion.Inverse(m_PandaRobot.transform.rotation) * liftOrientationWorld;

                // Use the 'pick_pose' field to send our single target pose.
                // We assume the ROS service can plan a trajectory from the current joint state to this pose.
                liftRequest.pick_pose = new PoseMsg
                {
                    position = liftPositionLocal.To<FLU>(),
                    orientation = liftOrientationLocal.To<FLU>()
                };

                // To ensure the service doesn't error, we can provide a dummy place_pose.
                liftRequest.place_pose = liftRequest.pick_pose;

                // Send the service request to MoveIt
                bool liftServiceCallComplete = false;
                PandaMoverServiceResponse liftResponse = null;

                m_Ros.SendServiceMessage<PandaMoverServiceResponse>(k_RosServiceName, liftRequest, (r) =>
                {
                    liftResponse = r;
                    liftServiceCallComplete = true;
                });

                // Wait for the service to return a trajectory
                yield return new WaitUntil(() => liftServiceCallComplete);

                if (liftResponse != null && liftResponse.trajectories.Length > 0)
                {
                    Debug.Log("Executing post-place lift trajectory.");
                    // Execute only the first trajectory returned, which should be the path to our lift pose.
                    yield return StartCoroutine(ExecuteTrajectory(liftResponse.trajectories[0]));
                    
                    // Wait a moment for the movement to complete
                    yield return new WaitForSeconds(0.5f / m_SpeedFactor);
                }
                else
                {
                    Debug.LogError("No trajectory returned from MoverService for the lift phase.");
                }
            }
        }
        
        enum Poses
        {
            PreGrasp,
            Grasp,
            PickUp,
            Place
        }

        /// <summary>
        /// Execute a single trajectory
        /// </summary>
        IEnumerator ExecuteTrajectory(RobotTrajectoryMsg trajectory)
        {
            if (trajectory.joint_trajectory.points.Length == 0)
            {
                yield break;
            }

            var jointTrajectory = trajectory.joint_trajectory;
            
            // Execute each trajectory point sequentially
            foreach (var trajectoryPoint in jointTrajectory.points)
            {
                var jointPositions = trajectoryPoint.positions;
                
                // Set the joint values for every joint (convert radians to degrees for Unity)
                for (var joint = 0; joint < k_NumRobotJoints && joint < jointPositions.Length; joint++)
                {
                    var jointAngleDegrees = (float)jointPositions[joint] * Mathf.Rad2Deg;
                    SetJointPosition(joint, jointAngleDegrees);
                }

                // Wait for robot to achieve pose for all joint assignments
                yield return new WaitForSeconds(0.1f / m_SpeedFactor);
            }
        }
        
        /// <summary>
        /// Handle gripper open/close actions using ArticulationBody control
        /// </summary>
        IEnumerator HandleGripperAction(bool close)
        {
            if (m_LeftGripper != null && m_RightGripper != null)
            {
                if (close)
                {
                    MoveGripper(m_GripperClosedPosition, m_GripperClosedPosition);
                }
                else
                {
                    MoveGripper(m_GripperOpenPosition, m_GripperOpenPosition);
                }
                
                yield return new WaitForSeconds(0.5f);
            }
            else
            {
                // Fallback to simulated approach if gripper ArticulationBodies not found
                if (close && m_Target != null)
                {
                    var endEffector = GetEndEffectorTransform();
                    if (endEffector != null)
                    {
                        m_Target.transform.SetParent(endEffector);
                        m_Target.transform.localPosition = Vector3.zero;
                    }
                }
                else if (!close && m_Target != null)
                {
                    m_Target.transform.SetParent(null);
                }

                yield return new WaitForSeconds(0.5f);
            }
        }

        #region Helper Functions

        PandaMoveitJointsMsg GetCurrentJointState()
        {
            var joints = new PandaMoveitJointsMsg();
            var positions = GetCurrentJointPositions();
            joints.joints = new double[k_NumRobotJoints];
            
            for (int i = 0; i < k_NumRobotJoints; i++)
            {
                joints.joints[i] = positions[i];
            }

            return joints;
        }

        double[] GetCurrentJointPositions()
        {
            var positions = new double[k_NumRobotJoints];
            for (int i = 0; i < k_NumRobotJoints; i++)
            {
                if (m_JointArticulationBodies[i] != null)
                {
                    var articulationBody = m_JointArticulationBodies[i].GetComponent<ArticulationBody>();
                    if (articulationBody != null)
                    {
                        positions[i] = articulationBody.jointPosition[0];
                    }
                    else
                    {
                        positions[i] = m_JointArticulationBodies[i].GetPosition();
                    }
                }
                else
                {
                    Debug.LogWarning($"Joint {i} ArticulationBody is null!");
                    positions[i] = 0.0;
                }
            }
            
            return positions;
        }

        void SetJointPosition(int jointIndex, float positionDegrees)
        {
            if (jointIndex < k_NumRobotJoints)
            {
                var articulationBody = m_JointArticulationBodies[jointIndex].GetComponent<ArticulationBody>();
                if (articulationBody != null)
                {
                    var xDrive = articulationBody.xDrive;
                    xDrive.target = positionDegrees;
                    articulationBody.xDrive = xDrive;
                }
            }
        }

        Transform GetEndEffectorTransform()
        {
            return m_PandaRobot.transform.Find("world/panda_link0/panda_link1/panda_link2/panda_link3/panda_link4/panda_link5/panda_link6/panda_link7/panda_link8/panda_hand/panda_hand_tcp");
        }

        public void MoveGripper(float leftTarget, float rightTarget)
        {
            if (m_LeftGripper != null && m_RightGripper != null)
            {
                var leftDrive = m_LeftGripper.xDrive;
                var rightDrive = m_RightGripper.xDrive;

                leftDrive.target = leftTarget;
                rightDrive.target = rightTarget;

                m_LeftGripper.xDrive = leftDrive;
                m_RightGripper.xDrive = rightDrive;
            }
        }

        Quaternion GetPickOrientation(Transform targetTransform)
        {
            if (targetTransform == null)
                return m_BaseGripperOrientation;

            var cubeYRotation = targetTransform.rotation.eulerAngles.y;
            var snappedYRotation = Mathf.Round(cubeYRotation / 90f) * 90f;
            var gripperYaw = snappedYRotation % 360f;
            
            return Quaternion.Euler(180, gripperYaw, 0);
        }
        
        #endregion
    }
}
