#!/usr/bin/env python

"""
MoveIt service for Franka Panda robot pick and place operations
Based on the niryo_moveit package but adapted for the 7-DOF Panda arm
Updated with best practices from MoveIt Python tutorial
"""

from __future__ import print_function

import rospy
import sys
import copy
import math
import moveit_commander

import moveit_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from panda_moveit.srv import PandaMoverService, PandaMoverServiceRequest, PandaMoverServiceResponse

# Joint names for Franka Panda (7 DOF)
joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']

def plan_trajectory(move_group, destination_pose, start_joint_angles): 
    """
    Given the start angles of the robot, plan a trajectory that ends at the destination pose.
    Uses modern MoveIt planning interface with better error handling.
    """
    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)

    move_group.set_pose_target(destination_pose)
    
    # Plan with modern interface
    success, plan, planning_time, error_code = move_group.plan()
    
    # Always clear pose targets after planning (best practice)
    move_group.clear_pose_targets()

    if not success:
        exception_str = """
            Trajectory could not be planned for a destination of {} with starting joint angles {}.
            Error code: {}. Planning time: {:.2f}s. 
            Please make sure target and destination are reachable by the robot.
        """.format(destination_pose, start_joint_angles, error_code, planning_time)
        raise Exception(exception_str)

    return plan

def plan_pick_and_place(req):
    """
    Creates a pick and place plan using the four states below.
    
    1. Pre Grasp - position gripper directly above target object
    2. Grasp - lower gripper so that fingers are on either side of object
    3. Pick Up - raise gripper back to the pre grasp position
    4. Place - move gripper to desired placement position

    Gripper behaviour is handled outside of this trajectory planning.
        - Gripper close occurs after 'grasp' position has been achieved
        - Gripper open occurs after 'place' position has been achieved
    """
    response = PandaMoverServiceResponse()

    group_name = "panda_manipulator"  # MoveIt planning group for Panda manipulator (includes gripper)
    move_group = moveit_commander.MoveGroupCommander(group_name)

    current_robot_joint_configuration = req.joints_input.joints

    # Pre grasp - position gripper directly above target object
    pre_grasp_pose = plan_trajectory(move_group, req.pick_pose, current_robot_joint_configuration)
    
    # If the trajectory has no points, planning has failed and we return an empty response
    if not pre_grasp_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = pre_grasp_pose.joint_trajectory.points[-1].positions

    # Grasp - lower gripper so that fingers are on either side of object
    pick_pose = copy.deepcopy(req.pick_pose)
    pick_pose.position.z -= 0.05  # Static value coming from Unity, TODO: pass along with request
    grasp_pose = plan_trajectory(move_group, pick_pose, previous_ending_joint_angles)
    
    if not grasp_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = grasp_pose.joint_trajectory.points[-1].positions

    # Pick Up - raise gripper back to the pre grasp position
    pick_up_pose = plan_trajectory(move_group, req.pick_pose, previous_ending_joint_angles)
    
    if not pick_up_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = pick_up_pose.joint_trajectory.points[-1].positions

    # Place - move gripper to desired placement position
    place_pose = plan_trajectory(move_group, req.place_pose, previous_ending_joint_angles)

    if not place_pose.joint_trajectory.points:
        return response

    # If trajectory planning worked for all pick and place stages, add plan to response
    response.trajectories.append(pre_grasp_pose)
    response.trajectories.append(grasp_pose)
    response.trajectories.append(pick_up_pose)
    response.trajectories.append(place_pose)

    move_group.clear_pose_targets()

    return response

def moveit_server():
    """
    Initialize the MoveIt service server for Panda robot
    """
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('panda_moveit_server')

    s = rospy.Service('panda_moveit', PandaMoverService, plan_pick_and_place)
    print("Panda MoveIt service ready to plan pick and place trajectories")
    rospy.spin()

if __name__ == "__main__":
    moveit_server()
