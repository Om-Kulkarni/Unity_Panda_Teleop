#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from panda_moveit.msg import PandaMoveitJoints

# List of your robot's joint names (must match your URDF)
JOINT_NAMES = [
    'panda_joint1', 'panda_joint2', 'panda_joint3',
    'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
]

def panda_joints_callback(msg):
    # msg is of type PandaMoveitJoints
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = JOINT_NAMES
    joint_state.position = msg.joints
    pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
    pub.publish(joint_state)

def main():
    rospy.init_node('unity_joint_state_publisher')
    rospy.Subscriber('/unity_panda_joint_states', PandaMoveitJoints, panda_joints_callback)
    rospy.loginfo('unity_joint_state_publisher: Listening for /unity_panda_joint_states and publishing /joint_states')
    rospy.spin()

if __name__ == '__main__':
    main()