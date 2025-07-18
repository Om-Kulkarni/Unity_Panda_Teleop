using System.Collections;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

using RosMessageTypes.PandaMoveit;
using RosMessageTypes.Geometry;
using Unity.Robotics.UrdfImporter;

public class PandaMovetoPoint : MonoBehaviour
{
    public bool initializationDone = false; // Set to true when the robot initialization is complete.
    [Header("References")]
    public GameObject pandaRobot;
    public Transform targetTransform;
    [Header("Trajectory Planner Service Name")]
    public string plannerServiceName = "panda_trajectory_planner";

    ROSConnection ros;
    Coroutine trackingCoroutine;

    // Joint link names for finding robot joints
    private static readonly string[] LinkNames =
    {
        "world/panda_link0/panda_link1", "/panda_link2", "/panda_link3", "/panda_link4", 
        "/panda_link5", "/panda_link6", "/panda_link7", "/panda_link8"
    };

    private UrdfJointRevolute[] jointArticulationBodies;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<PandaTrajectoryPlannerRequest, PandaTrajectoryPlannerResponse>(plannerServiceName);
        InitializeJoints();
    }

    void InitializeJoints()
    {
        jointArticulationBodies = new UrdfJointRevolute[LinkNames.Length];
        var linkName = string.Empty;
        for (var i = 0; i < LinkNames.Length; i++)
        {
            linkName += LinkNames[i];
            var jointTransform = pandaRobot.transform.Find(linkName);
            if (jointTransform != null)
            {
                jointArticulationBodies[i] = jointTransform.GetComponent<UrdfJointRevolute>();
            }
            else
            {
                jointArticulationBodies[i] = null;
            }
        }
    }

    void Update()
    {
        if (initializationDone && targetTransform != null && pandaRobot != null && trackingCoroutine == null)
        {
            trackingCoroutine = StartCoroutine(TrackTargetContinuously());
        }
    }

    IEnumerator TrackTargetContinuously()
    {
        while (true)
        {
            yield return StartCoroutine(MoveToPointTrajectory());
            // Optionally, add a small delay to avoid spamming the service
            yield return new WaitForSeconds(0.1f);
        }
    }

    IEnumerator MoveToPointTrajectory()
    {
        var req = new PandaTrajectoryPlannerRequest();
        Vector3 relPos = targetTransform.position - pandaRobot.transform.position;
        // Gripper facing down: 180 deg about X axis in Unity
        Quaternion gripperDown = Quaternion.Euler(180f, 0f, 0f);
        req.target_pose = new PoseMsg
        {
            position = relPos.To<FLU>(),
            orientation = gripperDown.To<FLU>()
        };
        // Get current joint values from robot using jointArticulationBodies
        int numJoints = jointArticulationBodies.Length;
        double[] jointVals = new double[numJoints];
        for (int i = 0; i < numJoints; i++)
        {
            if (jointArticulationBodies[i] != null)
            {
                var articulationBody = jointArticulationBodies[i].GetComponent<ArticulationBody>();
                if (articulationBody != null)
                {
                    jointVals[i] = articulationBody.jointPosition[0];
                }
                else
                {
                    jointVals[i] = jointArticulationBodies[i].GetPosition();
                }
            }
            else
            {
                jointVals[i] = 0.0;
            }
        }
        req.current_joints = new PandaMoveitJointsMsg { joints = jointVals };
        req.planning_group = "panda_arm";

        bool done = false;
        PandaTrajectoryPlannerResponse resp = null;
        ros.SendServiceMessage<PandaTrajectoryPlannerResponse>(plannerServiceName, req, (r) => { resp = r; done = true; });
        yield return new WaitUntil(() => done);
        if (resp != null && resp.success && resp.trajectory != null && resp.trajectory.joint_trajectory != null && resp.trajectory.joint_trajectory.points.Length > 0)
        {
            Debug.Log("Trajectory received. Executing " + resp.trajectory.joint_trajectory.points.Length + " points.");
            // For each point in the trajectory, apply the joint values with a small delay
            var points = resp.trajectory.joint_trajectory.points;
            double prevTime = 0.0;
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
                // Wait for the delta between this and the previous point
                double thisTime = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
                float waitTime = (float)(thisTime - prevTime);
                prevTime = thisTime;
                if (waitTime > 0f)
                    yield return new WaitForSeconds(waitTime);
                else
                    yield return new WaitForSeconds(0.01f); // fallback delay
            }
        }
        else
        {
            Debug.LogWarning("Trajectory planning failed: " + (resp != null ? resp.error_message : "No response"));
        }
    }
}
