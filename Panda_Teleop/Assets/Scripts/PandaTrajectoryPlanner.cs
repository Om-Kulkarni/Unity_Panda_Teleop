using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter;
using RosMessageTypes.PandaMoveit;  // Updated to use PandaMoveit messages
using RosMessageTypes.Geometry;
using RosMessageTypes.Trajectory;
using RosMessageTypes.Moveit;       // For RobotTrajectoryMsg

namespace Unity.Robotics.PickAndPlace
{
    /// <summary>
    ///     Trajectory planner for the Franka Panda robot using MoveIt
    /// </summary>
    public class PandaTrajectoryPlanner : MonoBehaviour
    {
        const int k_NumRobotJoints = 7;
        const string k_RosServiceName = "panda_moveit";
        
        public static readonly string[] LinkNames =
        {
            "world/panda_link0/panda_link1", "/panda_link2", "/panda_link3", "/panda_link4", 
            "/panda_link5", "/panda_link6", "/panda_link7"
        };

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
        [Tooltip("Speed factor for trajectory execution (0.1 = 10% speed)")]
        float m_SpeedFactor = 0.5f;

        [SerializeField]
        [Tooltip("Height above target for approach (in meters)")]
        float m_PickOffsetHeight = 0.15f;

        [SerializeField]
        [Tooltip("Additional offset for gripper TCP to prevent collision with table (in meters)")]
        float m_GripperTcpOffset = 0.05f;

        [SerializeField]
        [Tooltip("MoveIt planning group to use for trajectory planning")]
        string m_PlanningGroup = "panda_manipulator";
        public string PlanningGroup => m_PlanningGroup;

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
        ///     Execute a full pick and place operation
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
        ///     Execute pick and place operation as a coroutine
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
            
            // Pick Pose (with offset)
            var pickPosition = m_Target.transform.position + m_PickPoseOffset;
            var pickPositionRelativeToRobot = pickPosition - m_PandaRobot.transform.position;
            var pickOrientation = GetPickOrientation(m_Target.transform);
            var pickPose = new PoseMsg
            {
                position = pickPositionRelativeToRobot.To<FLU>(),
                orientation = pickOrientation.To<FLU>()
            };
            request.pick_pose = pickPose;
            
            // Place Pose (with offset) - use same orientation as pick for consistency
            var placePosition = m_TargetPlacement.transform.position + m_PickPoseOffset;
            var placePositionRelativeToRobot = placePosition - m_PandaRobot.transform.position;
            var placePose = new PoseMsg
            {
                position = placePositionRelativeToRobot.To<FLU>(),
                orientation = pickOrientation.To<FLU>()
            };
            request.place_pose = placePose;

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
        ///     Execute the trajectories returned by MoveIt
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
                        yield return StartCoroutine(HandleGripperAction(true));
                    }
                    
                    // Wait for the robot to achieve the final pose from joint assignment
                    yield return new WaitForSeconds(0.5f / m_SpeedFactor);
                }

                // All trajectories have been executed, open the gripper to place the target cube
                yield return StartCoroutine(HandleGripperAction(false));
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
        ///     Execute a single trajectory
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
        ///     Handle gripper open/close actions using ArticulationBody control
        /// </summary>
        IEnumerator HandleGripperAction(bool close)
        {
            if (m_LeftGripper != null && m_RightGripper != null)
            {
                if (close)
                {
                    // Close gripper - use simple approach like Niryo
                    MoveGripper(m_GripperClosedPosition, m_GripperClosedPosition);
                }
                else
                {
                    // Open gripper
                    MoveGripper(m_GripperOpenPosition, m_GripperOpenPosition);
                }
                
                // Use Niryo's timing approach
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
                        // Use jointPosition[0] like the working TrajectoryPlanner
                        positions[i] = articulationBody.jointPosition[0];
                    }
                    else
                    {
                        // Fallback to UrdfJointRevolute method
                        positions[i] = m_JointArticulationBodies[i].GetPosition();
                    }
                }
                else
                {
                    Debug.LogWarning($"Joint {i} ArticulationBody is null!");
                    positions[i] = 0.0; // Default to 0 if joint is missing
                }
            }
            
            // Debug log to verify joint positions
            Debug.Log($"Current joint positions (rad): [{string.Join(", ", positions.Select(p => p.ToString("F3")))}]");
            
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

        PoseMsg GetPoseMsg(Transform transform)
        {
            var pose = new PoseMsg();
            
            var relativePosition = transform.position - m_PandaRobot.transform.position;
            pose.position = relativePosition.To<FLU>();
            pose.orientation = transform.rotation.To<FLU>();

            return pose;
        }

        /// <summary>
        ///     Get the end-effector transform for the panda_manipulator group
        /// </summary>
        Transform GetEndEffectorTransform()
        {
            // Try to find the TCP (Tool Center Point) of the panda_manipulator group
            var endEffector = m_PandaRobot.transform.Find("world/panda_link0/panda_link1/panda_link2/panda_link3/panda_link4/panda_link5/panda_link6/panda_link7/panda_link8/panda_hand/panda_hand_tcp");
            
            if (endEffector != null)
                return endEffector;
            
            // Fallback to panda_hand if panda_hand_tcp not found
            endEffector = m_PandaRobot.transform.Find("world/panda_link0/panda_link1/panda_link2/panda_link3/panda_link4/panda_link5/panda_link6/panda_link7/panda_link8/panda_hand");
            
            if (endEffector != null)
                return endEffector;
            
            // Final fallback to panda_link8 (panda_arm group tip)
            endEffector = m_PandaRobot.transform.Find("world/panda_link0/panda_link1/panda_link2/panda_link3/panda_link4/panda_link5/panda_link6/panda_link7/panda_link8");
            
            return endEffector;
        }

        /// <summary>
        ///     Reset the pick and place state
        /// </summary>
        public void ResetPickAndPlace()
        {
            m_IsExecutingTrajectory = false;
            m_IsPickAndPlaceComplete = false;
            if (m_Target != null)
            {
                m_Target.transform.SetParent(null);
            }
            // Open gripper when resetting
            MoveGripper(m_GripperOpenPosition, m_GripperOpenPosition);
        }

        /// <summary>
        ///     Move the Panda gripper to specified positions
        /// </summary>
        /// <param name="leftTarget">Target position for left gripper</param>
        /// <param name="rightTarget">Target position for right gripper</param>
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

        // Public methods for UI
        public void SetTarget(GameObject target) => m_Target = target;
        public void SetTargetPlacement(GameObject target) => m_TargetPlacement = target;

        /// <summary>
        ///     Calculate pick orientation based on cube's orientation for optimal grip alignment
        /// </summary>
        /// <param name="targetTransform">The transform of the target object to pick</param>
        /// <returns>Quaternion representing the optimal pick orientation</returns>
        Quaternion GetPickOrientation(Transform targetTransform)
        {
            if (targetTransform == null)
                return m_BaseGripperOrientation;

            // Snap cube rotation to nearest 90-degree increment for consistent grip alignment
            var cubeYRotation = targetTransform.rotation.eulerAngles.y;
            var snappedYRotation = Mathf.Round(cubeYRotation / 90f) * 90f;

            // Add 45° offset to align gripper fingers with cube faces
            var gripperYaw = (snappedYRotation + 45f) % 360f;
            
            Debug.Log($"Cube Y: {cubeYRotation:F1}°, Snapped: {snappedYRotation:F1}°, Gripper Yaw: {gripperYaw:F1}°");
            
            return Quaternion.Euler(180, gripperYaw, 0);
        }
    }
}
