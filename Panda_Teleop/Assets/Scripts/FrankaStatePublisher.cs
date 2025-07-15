using System;
using System.Collections;
using RosMessageTypes.Franka;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter;
using UnityEngine;

public class FrankaStatePublisher : MonoBehaviour
{
    const int k_NumRobotJoints = 7;

    public static readonly string[] LinkNames =
        { "world/panda_link0/panda_link1", "/panda_link2", "/panda_link3", "/panda_link4", "/panda_link5", "/panda_link6", "/panda_link7" };

    // Variables required for ROS communication
    [SerializeField]
    string m_TopicName = "/franka_state";

    [SerializeField]
    GameObject m_FrankaRobot;

    [SerializeField]
    Transform m_EndEffector; // Reference to the end-effector transform

    [SerializeField]
    float m_PublishRate = 5.0f; // Publish every 5 seconds

    // Robot Joints
    UrdfJointRevolute[] m_JointArticulationBodies;

    // ROS Connector
    ROSConnection m_Ros;

    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<FrankaStateMsg>(m_TopicName);

        m_JointArticulationBodies = new UrdfJointRevolute[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += LinkNames[i];
            m_JointArticulationBodies[i] = m_FrankaRobot.transform.Find(linkName).GetComponent<UrdfJointRevolute>();
        }

        // Start the automatic publishing coroutine
        StartCoroutine(PublishPeriodically());
    }

    // Coroutine to publish Franka state every m_PublishRate seconds
    IEnumerator PublishPeriodically()
    {
        while (true)
        {
            yield return new WaitForSeconds(m_PublishRate);
            Publish();
        }
    }

    public void Publish()
    {
        var frankaStateMessage = new FrankaStateMsg();

        // Set joint positions (q) and velocities (dq)
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            frankaStateMessage.q[i] = m_JointArticulationBodies[i].GetPosition();
            frankaStateMessage.dq[i] = m_JointArticulationBodies[i].GetVelocity();
        }

        // Set end-effector transformation matrix (O_T_EE) if end-effector is assigned
        if (m_EndEffector != null)
        {
            var endEffectorMatrix = m_EndEffector.localToWorldMatrix;
            // Convert Unity's column-major 4x4 matrix to row-major array
            frankaStateMessage.O_T_EE[0] = endEffectorMatrix.m00;
            frankaStateMessage.O_T_EE[1] = endEffectorMatrix.m01;
            frankaStateMessage.O_T_EE[2] = endEffectorMatrix.m02;
            frankaStateMessage.O_T_EE[3] = endEffectorMatrix.m03;
            frankaStateMessage.O_T_EE[4] = endEffectorMatrix.m10;
            frankaStateMessage.O_T_EE[5] = endEffectorMatrix.m11;
            frankaStateMessage.O_T_EE[6] = endEffectorMatrix.m12;
            frankaStateMessage.O_T_EE[7] = endEffectorMatrix.m13;
            frankaStateMessage.O_T_EE[8] = endEffectorMatrix.m20;
            frankaStateMessage.O_T_EE[9] = endEffectorMatrix.m21;
            frankaStateMessage.O_T_EE[10] = endEffectorMatrix.m22;
            frankaStateMessage.O_T_EE[11] = endEffectorMatrix.m23;
            frankaStateMessage.O_T_EE[12] = endEffectorMatrix.m30;
            frankaStateMessage.O_T_EE[13] = endEffectorMatrix.m31;
            frankaStateMessage.O_T_EE[14] = endEffectorMatrix.m32;
            frankaStateMessage.O_T_EE[15] = endEffectorMatrix.m33;
        }

        // Set simulation time
        frankaStateMessage.time = Time.time;

        // Set robot mode (assuming MOVE mode during simulation)
        frankaStateMessage.robot_mode = FrankaStateMsg.ROBOT_MODE_MOVE;

        // Set control command success rate (assuming 100% in simulation)
        frankaStateMessage.control_command_success_rate = 1.0;

        // Finally send the message to ROS
        m_Ros.Publish(m_TopicName, frankaStateMessage);
    }
}
