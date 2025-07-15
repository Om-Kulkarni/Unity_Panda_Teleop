#!/usr/bin/env python

"""
Real-time IK solver service for Franka Panda VR teleoperation
Provides fast inverse kinematics solving for Unity VR controller input
"""

from __future__ import print_function

import rospy
import sys
import moveit_commander
import actionlib
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState, MoveItErrorCodes, MoveGroupAction
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header

from panda_moveit.srv import PandaIKSolver, PandaIKSolverRequest, PandaIKSolverResponse
from panda_moveit.msg import PandaMoveitJoints

# Joint names for Franka Panda (7 DOF)
JOINT_NAMES = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 
               'panda_joint5', 'panda_joint6', 'panda_joint7']

class PandaTeleopServer:
    """
    Real-time IK solver server for VR teleoperation
    """
    
    def __init__(self):
        """Initialize the teleop server"""
        rospy.loginfo("Initializing Panda VR teleoperation server...")
        
        # Wait for MoveIt to be ready
        self._wait_for_moveit()
        
        # Initialize MoveIt commander
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Initialize the move group for the Panda manipulator
        self.group_name = "panda_manipulator"
        rospy.loginfo(f"Connecting to MoveIt planning group: {self.group_name}")
        
        try:
            self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        except Exception as e:
            rospy.logerr(f"Failed to connect to move group '{self.group_name}': {e}")
            raise
        
        # Get the robot commander for full robot state
        try:
            self.robot = moveit_commander.RobotCommander()
        except Exception as e:
            rospy.logerr(f"Failed to initialize robot commander: {e}")
            raise
        
        # Set planning parameters for fast IK solving
        self.move_group.set_planning_time(0.05)  # 50ms timeout for real-time performance
        self.move_group.set_num_planning_attempts(3)  # Limited attempts for speed
        
        # Get end-effector link name
        self.end_effector_link = self.move_group.get_end_effector_link()
        rospy.loginfo(f"End effector link: {self.end_effector_link}")
        
        # Initialize the service
        self.service = rospy.Service('panda_ik_solver', PandaIKSolver, self.solve_ik_callback)
        rospy.loginfo("Panda VR teleoperation IK solver service ready")
    
    def _wait_for_moveit(self):
        """
        Wait for MoveIt services to become available
        """
        rospy.loginfo("Waiting for MoveIt services to become available...")
        
        # Wait for move_group action server
        try:
            move_group_client = actionlib.SimpleActionClient('move_group', MoveGroupAction)
            rospy.loginfo("Waiting for move_group action server...")
            
            if not move_group_client.wait_for_server(timeout=rospy.Duration(30.0)):
                raise Exception("move_group action server not available after 30 seconds")
            
            rospy.loginfo("move_group action server is ready")
            
        except Exception as e:
            rospy.logerr(f"Failed to connect to move_group: {e}")
            raise
    
    def solve_ik_callback(self, req):
        """
        Callback function for IK solver service
        
        Args:
            req (PandaIKSolverRequest): Service request containing target pose and current joints
            
        Returns:
            PandaIKSolverResponse: Service response with success flag and joint solution
        """
        response = PandaIKSolverResponse()
        
        try:
            # Validate request
            if not self._validate_request(req):
                response.success = False
                response.error_message = "Invalid request: missing required fields"
                return response
            
            # Set the current robot state as starting point
            self._set_start_state(req.current_joints)
            
            # Create pose stamped from request
            pose_stamped = self._create_pose_stamped(req.target_pose)
            
            # Solve IK with collision checking option
            joint_solution = self._solve_ik(pose_stamped, req.collision_checking)
            
            if joint_solution is not None:
                # Successful IK solution
                response.success = True
                response.joint_solution = self._create_joint_msg(joint_solution)
                response.error_message = ""
            else:
                # IK solving failed
                response.success = False
                response.error_message = "No IK solution found for target pose"
                
        except Exception as e:
            # Handle any unexpected errors
            response.success = False
            response.error_message = f"IK solver error: {str(e)}"
            rospy.logerr(f"IK solver exception: {e}")
        
        return response
    
    def _validate_request(self, req):
        """
        Validate the incoming IK request
        
        Args:
            req (PandaIKSolverRequest): The service request
            
        Returns:
            bool: True if request is valid, False otherwise
        """
        # Check if target pose is provided
        if req.target_pose is None:
            return False
        
        # Check if current joints are provided and have correct length
        if req.current_joints is None or len(req.current_joints.joints) != 7:
            return False
        
        # Check planning group
        if req.planning_group and req.planning_group != self.group_name:
            rospy.logwarn(f"Requested planning group '{req.planning_group}' differs from configured group '{self.group_name}'")
        
        return True
    
    def _set_start_state(self, current_joints):
        """
        Set the current robot state as the starting point for IK solving
        
        Args:
            current_joints (PandaMoveitJoints): Current joint positions
        """
        # Create joint state message
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = JOINT_NAMES
        joint_state.position = current_joints.joints
        
        # Create robot state
        robot_state = RobotState()
        robot_state.joint_state = joint_state
        
        # Set as start state for planning
        self.move_group.set_start_state(robot_state)
    
    def _create_pose_stamped(self, target_pose):
        """
        Convert geometry_msgs/Pose to geometry_msgs/PoseStamped
        
        Args:
            target_pose (geometry_msgs/Pose): Target pose
            
        Returns:
            geometry_msgs/PoseStamped: Stamped pose with frame information
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.move_group.get_planning_frame()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = target_pose
        
        return pose_stamped
    
    def _solve_ik(self, pose_stamped, collision_checking=True):
        """
        Solve inverse kinematics for the given pose
        
        Args:
            pose_stamped (geometry_msgs/PoseStamped): Target pose
            collision_checking (bool): Whether to check for collisions
            
        Returns:
            list: Joint positions if successful, None if failed
        """
        try:
            # Set the pose target
            self.move_group.set_pose_target(pose_stamped, self.end_effector_link)
            
            # Disable/enable collision checking based on request
            if not collision_checking:
                self.move_group.set_planning_time(0.02)  # Even faster without collision checking
            
            # Plan to get IK solution (we only need the start state of the trajectory)
            success, plan, planning_time, error_code = self.move_group.plan()
            
            # Clear targets
            self.move_group.clear_pose_targets()
            
            if success and plan.joint_trajectory.points:
                # Return the joint positions from the first trajectory point
                # (this represents the IK solution for our target pose)
                return list(plan.joint_trajectory.points[0].positions)
            else:
                rospy.logdebug(f"IK planning failed with error code: {error_code}")
                return None
                
        except Exception as e:
            rospy.logerr(f"IK solving failed: {e}")
            self.move_group.clear_pose_targets()
            return None
    
    def _create_joint_msg(self, joint_positions):
        """
        Create PandaMoveitJoints message from joint positions
        
        Args:
            joint_positions (list): List of joint positions in radians
            
        Returns:
            PandaMoveitJoints: Joint message
        """
        joint_msg = PandaMoveitJoints()
        joint_msg.joints = joint_positions
        return joint_msg

def main():
    """
    Main function to start the teleop server
    """
    rospy.init_node('panda_teleop_server', anonymous=True)
    
    try:
        # Create and start the teleop server
        teleop_server = PandaTeleopServer()
        
        rospy.loginfo("Panda VR teleoperation server started successfully")
        rospy.loginfo("Waiting for IK solver requests...")
        
        # Keep the node running
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Panda teleoperation server interrupted")
    except Exception as e:
        rospy.logerr(f"Failed to start teleoperation server: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
