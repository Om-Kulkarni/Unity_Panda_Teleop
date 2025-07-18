using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter;
using RosMessageTypes.PandaMoveit;
using RosMessageTypes.Geometry;

namespace Unity.Robotics.PickAndPlace
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
        [SerializeField] private int numJoints = 7;
        
        // Robot joints
        private UrdfJointRevolute[] jointArticulationBodies;
        
        // Joint link names for finding robot joints
        private static readonly string[] LinkNames =
        {
            "world/panda_link0/panda_link1", "/panda_link2", "/panda_link3", "/panda_link4", 
            "/panda_link5", "/panda_link6", "/panda_link7"
        };
        [Header("MoveIt Integration")]
        [SerializeField] private string ikServiceName = "panda_ik_solver";
        [SerializeField] private bool enableCollisionChecking = true;
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
        
        // Timing
        private float lastUpdateTime;
        private Coroutine teleopCoroutine;

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
            
            if (endEffectorTransform == null)
                endEffectorTransform = GetEndEffectorTransform();
            
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
            // Auto-find the VR handle if not assigned
            if (endEffectorInteractable == null && endEffectorTransform != null)
            {
                // Look for the VR handle child object
                Transform vrHandle = endEffectorTransform.Find("EndEffector_VR_handle");
                if (vrHandle != null)
                {
                    endEffectorInteractable = vrHandle.GetComponent<XRSimpleInteractable>();
                    if (endEffectorInteractable == null)
                    {
                        Debug.LogWarning("XRSimpleInteractable component not found on EndEffector_VR_handle! Please add it in the inspector.");
                        return;
                    }
                }
                else
                {
                    Debug.LogError("EndEffector_VR_handle child object not found! Please create it as a child of the end-effector.");
                    return;
                }
            }

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
            
            // Register MoveIt IK service
            rosConnection.RegisterRosService<PandaIKSolverRequest, PandaIKSolverResponse>(ikServiceName);
        }

        void OnGrabStarted(SelectEnterEventArgs args)
        {
            isGrabbed = true;
            vrControllerTransform = args.interactorObject.transform;
            
            // Calculate grab offset relative to the actual end-effector (not the VR handle)
            // This ensures smooth teleoperation regardless of where the VR handle is positioned
            grabOffset = endEffectorTransform.position - vrControllerTransform.position;
            grabRotationOffset = Quaternion.Inverse(vrControllerTransform.rotation) * endEffectorTransform.rotation;
            
            // Start teleoperation coroutine
            if (teleopCoroutine != null)
                StopCoroutine(teleopCoroutine);
            teleopCoroutine = StartCoroutine(TeleopLoop());
            
            Debug.Log("VR Teleoperation Started - controlling actual end-effector through VR handle");
        }

        void OnGrabEnded(SelectExitEventArgs args)
        {
            isGrabbed = false;
            vrControllerTransform = null;
            
            // Stop teleoperation
            if (teleopCoroutine != null)
            {
                StopCoroutine(teleopCoroutine);
                teleopCoroutine = null;
            }
            
            Debug.Log("VR Teleoperation Stopped");
        }

        IEnumerator TeleopLoop()
        {
            float updateInterval = 1f / updateRate;
            
            while (isGrabbed && vrControllerTransform != null)
            {
                // Calculate target pose from VR controller
                UpdateTargetPose();
                
                // Check if movement is significant enough to send
                if (ShouldSendUpdate())
                {
                    yield return StartCoroutine(SendMoveItIKRequest());
                }
                
                yield return new WaitForSeconds(updateInterval);
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

        IEnumerator SendMoveItIKRequest()
        {
            // Create IK request using the generated message type
            var ikRequest = new PandaIKSolverRequest();
            
            // Set target pose relative to robot base
            Vector3 relativePosition = targetPosition - pandaRobot.transform.position;
            ikRequest.target_pose = new PoseMsg
            {
                position = relativePosition.To<FLU>(),
                orientation = targetRotation.To<FLU>()
            };
            
            // Set current joint state as starting point
            ikRequest.current_joints = GetCurrentJointState();
            ikRequest.planning_group = "panda_manipulator";
            ikRequest.collision_checking = enableCollisionChecking;
            
            // Send IK request
            bool responseReceived = false;
            PandaIKSolverResponse ikResponse = null;
            
            rosConnection.SendServiceMessage<PandaIKSolverResponse>(ikServiceName, ikRequest, (response) =>
            {
                ikResponse = response;
                responseReceived = true;
            });
            
            // Wait for response with timeout
            float timeout = 0.1f; // 100ms timeout for real-time performance
            float startTime = Time.time;
            
            yield return new WaitUntil(() => responseReceived || (Time.time - startTime) > timeout);
            
            if (responseReceived && ikResponse != null && ikResponse.success)
            {
                // Apply IK solution to robot using the joint_solution from the response
                ApplyJointSolutionFromMsg(ikResponse.joint_solution);
                
                // Update last sent pose
                lastSentPosition = targetPosition;
                lastSentRotation = targetRotation;
            }
            else if (!responseReceived)
            {
                Debug.LogWarning("IK service timeout - target may be unreachable");
            }
            else if (ikResponse != null && !ikResponse.success)
            {
                Debug.LogWarning($"IK solver failed: {ikResponse.error_message}");
            }
        }

        void ApplyJointSolutionFromMsg(PandaMoveitJointsMsg jointSolution)
        {
            // Apply joint positions to robot from PandaMoveitJointsMsg
            for (int i = 0; i < jointSolution.joints.Length && i < numJoints; i++)
            {
                float jointAngleDegrees = (float)jointSolution.joints[i] * Mathf.Rad2Deg;
                SetJointPosition(i, jointAngleDegrees);
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

        /// <summary>
        /// Get the end-effector transform for the panda_manipulator group
        /// </summary>
        Transform GetEndEffectorTransform()
        {
            if (pandaRobot == null) return null;
            
            // Try to find the TCP (Tool Center Point) transform
            Transform endEffector = pandaRobot.transform.Find("world/panda_link0/panda_link1/panda_link2/panda_link3/panda_link4/panda_link5/panda_link6/panda_link7/panda_link8/panda_hand/panda_hand_tcp");
            if (endEffector == null)
            {
                Debug.LogWarning("End-effector transform not found! Please ensure the robot is set up correctly.");
                return null;
            }
            return endEffector;
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
