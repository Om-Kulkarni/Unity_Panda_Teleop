#!/usr/bin/env python

import rospy
import sys
import moveit_commander
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import PoseStamped
from panda_moveit.srv import PandaTrajectoryPlanner, PandaTrajectoryPlannerResponse
from panda_moveit.msg import PandaMoveitJoints

JOINT_NAMES = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']

class PandaTeleopServer:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.group_name = "panda_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.end_effector_link = self.move_group.get_end_effector_link()
        self.move_group.set_planning_time(0.05)
        self.move_group.set_num_planning_attempts(3)
        self.service = rospy.Service('panda_trajectory_planner', PandaTrajectoryPlanner, self.solve_ik_callback)

    def solve_ik_callback(self, req):
        response = PandaTrajectoryPlannerResponse()
        # Set the robot's start state from current joints
        joint_state = JointState()
        joint_state.name = JOINT_NAMES
        joint_state.position = req.current_joints.joints
        robot_state = RobotState()
        robot_state.joint_state = joint_state
        self.move_group.set_start_state(robot_state)

        # Set the target pose
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.move_group.get_planning_frame()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = req.target_pose
        self.move_group.set_pose_target(pose_stamped, self.end_effector_link)

        # Plan a full trajectory to the target pose
        success, plan, planning_time, error_code = self.move_group.plan()
        self.move_group.clear_pose_targets()

        # Return the full trajectory in the response
        if success and plan.joint_trajectory.points:
            response.success = True
            response.trajectory = plan  # plan is a moveit_msgs/RobotTrajectory
            response.error_message = ""
        else:
            response.success = False
            response.trajectory = None
            response.error_message = "No trajectory found for target pose"
        return response

def main():
    rospy.init_node('panda_teleop_server', anonymous=True)
    teleop_server = PandaTeleopServer()
    rospy.spin()

if __name__ == "__main__":
    sys.exit(main())
