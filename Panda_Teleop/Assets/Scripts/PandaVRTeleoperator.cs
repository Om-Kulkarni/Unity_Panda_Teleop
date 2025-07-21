using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.XR;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter;
using RosMessageTypes.PandaMoveit;
using RosMessageTypes.Geometry;
using UnityEngine.InputSystem;

namespace Unity.Robotics.Teleoperation
{
    public class PandaVRTeleoperator : MonoBehaviour
    {
        [Header("Robot References")]
        [SerializeField] private GameObject pandaRobot;
        
        [Header("VR Interaction")]
        [SerializeField] private XRSimpleInteractable endEffectorInteractable;
        [SerializeField] private Transform endEffectorTransform;
        
        [Header("Control Parameters")]
        public float positionSmoothingFactor = 10f;
        public float rotationSmoothingFactor = 5f;
        public float updateRate = 30f; // Hz
        public float minimumMovementThreshold = 0.001f; // meters
        public float minimumRotationThreshold = 1f; // degrees
        
        [Header("Robot Configuration")]
        [SerializeField] private int numJoints = 8;

        [Header("Gripper Control")]
        [SerializeField] private ArticulationBody leftGripper;
        [SerializeField] private ArticulationBody rightGripper;
        [SerializeField] private InputActionReference triggerInputAction; // Assign the trigger action in inspector
        [SerializeField] private float gripperOpenPosition = 0.04f; // meters (adjust as needed)
        [SerializeField] private float gripperClosedPosition = 0.0f; // meters (adjust as needed)

        [Header("Visualization")]
        [Tooltip("Show XYZ axes at the target pose for debugging/visualization.")]
        public bool showTargetPoseGizmo = true;
        [Tooltip("Length of the XYZ axes for the target pose visualization.")]
        public float targetPoseAxisLength = 0.1f;
        // Draw XYZ axes at the target pose for visualization

        void Update()
        {
            // Gripper control: open when trigger held, close when released
            if (triggerInputAction != null && leftGripper != null && rightGripper != null)
            {
                float triggerValue = triggerInputAction.action.ReadValue<float>();
                float targetPosition = triggerValue > 0.5f ? gripperOpenPosition : gripperClosedPosition;
                SetGripperPosition(targetPosition);
            }
        }

        void SetGripperPosition(float position)
        {
            var leftDrive = leftGripper.xDrive;
            leftDrive.target = position * Mathf.Rad2Deg;
            leftGripper.xDrive = leftDrive;

            var rightDrive = rightGripper.xDrive;
            rightDrive.target = position * Mathf.Rad2Deg;
            rightGripper.xDrive = rightDrive;
        }

        void OnDrawGizmos()
        {
            if (!showTargetPoseGizmo) return;
            // Only draw if we have a valid target pose
            Gizmos.color = Color.red;
            Gizmos.DrawLine(targetPosition, targetPosition + targetRotation * Vector3.right * targetPoseAxisLength);
            Gizmos.color = Color.green;
            Gizmos.DrawLine(targetPosition, targetPosition + targetRotation * Vector3.up * targetPoseAxisLength);
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(targetPosition, targetPosition + targetRotation * Vector3.forward * targetPoseAxisLength);
        }
        
        // Robot joints
        private UrdfJointRevolute[] jointArticulationBodies;
        
        // Joint link names for finding robot joints
        private static readonly string[] LinkNames =
        {
            "world/panda_link0/panda_link1", "/panda_link2", "/panda_link3", "/panda_link4", 
            "/panda_link5", "/panda_link6", "/panda_link7", "/panda_link8"
        };
        [Header("MoveIt Integration")]
        [SerializeField] private string trajectoryServiceName = "panda_trajectory_planner";
        
        // [SerializeField] private bool enableCollisionChecking = true;
        private bool isGrabbed = false;
        private Vector3 targetPosition;
        private Quaternion targetRotation;
        private Vector3 lastSentPosition;
        private Quaternion lastSentRotation;
        private Transform vrControllerTransform;
        private Vector3 grabOffset;
        private Quaternion grabRotationOffset;
        
        // ROS connection
        private ROSConnection rosConnection;
        // Publisher for Unity joint states
        // private string unityJointStateTopic = "/unity_panda_joint_states";
        
        // Timing
        private Coroutine trackingCoroutine;

        void Start()
        {
            InitializeComponents();
            SetupVRInteraction();
            InitializeROS();
        }

        void InitializeComponents()
        {
            // Auto-find robot if not assigned
            if (pandaRobot == null)
            {
                Debug.LogWarning("Panda robot not assigned, trying to find it...");
                pandaRobot = GameObject.FindGameObjectWithTag("robot");
                if (pandaRobot == null)
                {
                    Debug.LogError("Could not find Panda robot! Please assign it in the inspector.");
                    return;
                }
            }
            
            // Initialize joint references
            InitializeJoints();
            
            // Initialize target pose to current end-effector pose
            if (endEffectorTransform != null)
            {
                targetPosition = endEffectorTransform.position;
                targetRotation = endEffectorTransform.rotation;
                lastSentPosition = targetPosition;
                lastSentRotation = targetRotation;
            }
        }

        void InitializeJoints()
        {
            jointArticulationBodies = new UrdfJointRevolute[numJoints];
            var linkName = string.Empty;
            
            for (var i = 0; i < numJoints; i++)
            {
                linkName += LinkNames[i];
                var jointTransform = pandaRobot.transform.Find(linkName);
                
                if (jointTransform != null)
                {
                    jointArticulationBodies[i] = jointTransform.GetComponent<UrdfJointRevolute>();
                    if (jointArticulationBodies[i] == null)
                    {
                        Debug.LogWarning($"UrdfJointRevolute component not found on joint {i} at path: {linkName}");
                    }
                }
                else
                {
                    Debug.LogError($"Could not find joint {i} at path: {linkName}");
                }
            }
        }

        void SetupVRInteraction()
        {
            if (endEffectorInteractable != null)
            {
                // Subscribe to select (grab) events
                endEffectorInteractable.selectEntered.AddListener(OnGrabStarted);
                endEffectorInteractable.selectExited.AddListener(OnGrabEnded);
                Debug.Log("VR interaction setup complete with EndEffector_VR_handle (XRBaseInteractable)");
            }
            else
            {
                Debug.LogError("Could not setup VR interaction - endEffectorInteractable is null!");
            }
        }

        void InitializeROS()
        {
            rosConnection = ROSConnection.GetOrCreateInstance();
            // Register MoveIt trajectory planning service
            rosConnection.RegisterRosService<PandaTrajectoryPlannerServiceRequest, PandaTrajectoryPlannerServiceResponse>(trajectoryServiceName);
        }

        void OnGrabStarted(SelectEnterEventArgs args)
        {
            isGrabbed = true;
            vrControllerTransform = args.interactorObject.transform;

            // Synchronize Unity's joint state before starting teleoperation
            lastSentPosition = endEffectorTransform.position;
            lastSentRotation = endEffectorTransform.rotation;

            // Calculate grab offset relative to the actual end-effector (not the VR handle)
            // This ensures smooth teleoperation regardless of where the VR handle is positioned
            grabOffset = endEffectorTransform.position - vrControllerTransform.position;
            grabRotationOffset = Quaternion.Inverse(vrControllerTransform.rotation) * endEffectorTransform.rotation;

            // Start teleoperation coroutine
            if (trackingCoroutine != null)
                StopCoroutine(trackingCoroutine);
            trackingCoroutine = StartCoroutine(TrackTargetContinuously());

            Debug.Log("VR Teleoperation Started - controlling actual end-effector through VR handle");
        }

        void OnGrabEnded(SelectExitEventArgs args)
        {
            isGrabbed = false;
            vrControllerTransform = null;
            
            // Stop teleoperation
            if (trackingCoroutine != null)
            {
                StopCoroutine(trackingCoroutine);
                trackingCoroutine = null;
            }
            
            Debug.Log("VR Teleoperation Stopped");
        }

        IEnumerator TrackTargetContinuously()
        {
            while (isGrabbed && vrControllerTransform != null)
            {
                UpdateTargetPose();
                yield return StartCoroutine(SendMoveItTrajectoryRequest());
                yield return new WaitForSeconds(0.02f); // Faster request rate
            }
        }

        void UpdateTargetPose()
        {
            if (vrControllerTransform == null) return;
            
            // Calculate smooth target position and rotation
            Vector3 desiredPosition = vrControllerTransform.position + grabOffset;
            Quaternion desiredRotation = vrControllerTransform.rotation * grabRotationOffset;
            
            // Apply smoothing
            targetPosition = Vector3.Lerp(targetPosition, desiredPosition, 
                Time.deltaTime * positionSmoothingFactor);
            targetRotation = Quaternion.Lerp(targetRotation, desiredRotation, 
                Time.deltaTime * rotationSmoothingFactor);
        }

        bool ShouldSendUpdate()
        {
            float positionDelta = Vector3.Distance(targetPosition, lastSentPosition);
            float rotationDelta = Quaternion.Angle(targetRotation, lastSentRotation);
            
            return positionDelta > minimumMovementThreshold || 
                   rotationDelta > minimumRotationThreshold;
        }


        IEnumerator SendMoveItTrajectoryRequest()
        {
            // Create trajectory planning request
            var req = new PandaTrajectoryPlannerServiceRequest();
            Vector3 relPos = targetPosition - pandaRobot.transform.position;
            Quaternion gripperDown = Quaternion.Euler(180f, 0f, 0f);
            req.target_pose = new PoseMsg
            {
                position = relPos.To<FLU>(),
                orientation = gripperDown.To<FLU>()
            };
            req.current_joints = GetCurrentJointState();
            req.planning_group = "panda_arm";

            bool done = false;
            PandaTrajectoryPlannerServiceResponse resp = null;
            rosConnection.SendServiceMessage<PandaTrajectoryPlannerServiceResponse>(trajectoryServiceName, req, (r) => { resp = r; done = true; });
            yield return new WaitUntil(() => done);
            if (resp != null && resp.success && resp.trajectory != null && resp.trajectory.joint_trajectory != null && resp.trajectory.joint_trajectory.points.Length > 0)
            {
                var points = resp.trajectory.joint_trajectory.points;
                foreach (var point in points)
                {
                    var joints = point.positions;
                    for (int i = 0; i < jointArticulationBodies.Length && i < joints.Length; i++)
                    {
                        if (jointArticulationBodies[i] != null)
                        {
                            var articulationBody = jointArticulationBodies[i].GetComponent<ArticulationBody>();
                            if (articulationBody != null)
                            {
                                var drive = articulationBody.xDrive;
                                drive.stiffness = 10000f;
                                drive.forceLimit = 1000f;
                                drive.target = (float)(joints[i]) * Mathf.Rad2Deg;
                                articulationBody.xDrive = drive;
                            }
                        }
                    }
                    // Maximum speed: no yield, all joint positions applied in one frame
                }
                lastSentPosition = targetPosition;
                lastSentRotation = targetRotation;
            }
            else
            {
                Debug.LogWarning("Trajectory planning failed: " + (resp != null ? resp.error_message : "No response"));
            }
        }

        IEnumerator ExecuteTrajectory(RosMessageTypes.Moveit.RobotTrajectoryMsg trajectory)
        {
            if (trajectory == null || trajectory.joint_trajectory == null || trajectory.joint_trajectory.points == null || trajectory.joint_trajectory.points.Length == 0)
                yield break;
            var jointTrajectory = trajectory.joint_trajectory;
            foreach (var point in jointTrajectory.points)
            {
                var jointPositions = point.positions;
                for (int i = 0; i < jointPositions.Length && i < numJoints; i++)
                {
                    float jointAngleDegrees = (float)jointPositions[i] * Mathf.Rad2Deg;
                    SetJointPosition(i, jointAngleDegrees);
                }
                // Wait for a short time to simulate smooth motion
                yield return new WaitForSeconds(0.05f);
            }
        }        

        void SetJointPosition(int jointIndex, float positionDegrees)
        {
            if (jointIndex < numJoints && jointArticulationBodies[jointIndex] != null)
            {
                var articulationBody = jointArticulationBodies[jointIndex].GetComponent<ArticulationBody>();
                if (articulationBody != null)
                {
                    var xDrive = articulationBody.xDrive;
                    xDrive.target = positionDegrees;
                    articulationBody.xDrive = xDrive;
                }
            }
        }

        PandaMoveitJointsMsg GetCurrentJointState()
        {
            var joints = new PandaMoveitJointsMsg();
            var positions = GetCurrentJointPositions();
            joints.joints = new double[numJoints];
            
            for (int i = 0; i < numJoints; i++)
            {
                joints.joints[i] = positions[i];
            }

            return joints;
        }

        double[] GetCurrentJointPositions()
        {
            var positions = new double[numJoints];
            for (int i = 0; i < numJoints; i++)
            {
                if (jointArticulationBodies[i] != null)
                {
                    var articulationBody = jointArticulationBodies[i].GetComponent<ArticulationBody>();
                    if (articulationBody != null)
                    {
                        // Use jointPosition[0] for revolute joints
                        positions[i] = articulationBody.jointPosition[0];
                    }
                    else
                    {
                        // Fallback to UrdfJointRevolute method
                        positions[i] = jointArticulationBodies[i].GetPosition();
                    }
                }
                else
                {
                    Debug.LogWarning($"Joint {i} ArticulationBody is null!");
                    positions[i] = 0.0; // Default to 0 if joint is missing
                }
            }
            
            return positions;
        }

        void OnEnable()
        {
            if (triggerInputAction != null)
                triggerInputAction.action.Enable();
        }

        void OnDisable()
        {
            if (triggerInputAction != null)
                triggerInputAction.action.Disable();
        }

        void OnDestroy()
        {
            // Clean up event subscriptions
            if (endEffectorInteractable != null)
            {
                endEffectorInteractable.selectEntered.RemoveListener(OnGrabStarted);
                endEffectorInteractable.selectExited.RemoveListener(OnGrabEnded);
            }
        }
    }
}
