#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
import actionlib
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveItIkDemo:
    def __init__(self):

        joint_state_topic = ['joint_states:=/iiwa/joint_states']
        moveit_commander.roscpp_initialize(joint_state_topic)
        
        
        rospy.init_node('moveit_ik_demo')
                
        
        arm = moveit_commander.MoveGroupCommander('arm')
        gripper = moveit_commander.MoveGroupCommander('gripper')
                
        
        end_effector_link = arm.get_end_effector_link()
                        
        
        reference_frame = 'iiwa_link_0'
        arm.set_pose_reference_frame(reference_frame)
                
        
	gripper_joints = ['finger_joint']

        arm.allow_replanning(True)

       
	# ----set gripper action client----
        gripper_client = actionlib.SimpleActionClient('iiwa/gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        gripper_client.wait_for_server()        
        rospy.loginfo('...connected.')
        
        
	# ----set accuracy----
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.01)

        gripper.set_goal_position_tolerance(0.01)
        gripper.set_goal_orientation_tolerance(0.05)


	# ----set arm poses----

	# pose1
	target_pose1 = PoseStamped()
	target_pose1.header.frame_id = reference_frame
	target_pose1.header.stamp = rospy.Time.now()   

	target_pose1.pose.position.x = 0.6
	target_pose1.pose.position.y = 0.0
	target_pose1.pose.position.z = 0.3
	target_pose1.pose.orientation.x = 0
	target_pose1.pose.orientation.y = 1
	target_pose1.pose.orientation.z = 0
	target_pose1.pose.orientation.w = 0
	
	# pose1
	target_pose2 = PoseStamped()
	target_pose2.header.frame_id = reference_frame
	target_pose2.header.stamp = rospy.Time.now()   

	target_pose2.pose.position.x = 0.0
	target_pose2.pose.position.y = 0.6
	target_pose2.pose.position.z = 0.3
	target_pose2.pose.orientation.x = 0
	target_pose2.pose.orientation.y = 1
	target_pose2.pose.orientation.z = 0
	target_pose2.pose.orientation.w = 0


	# ----set gripepr poses----
	gripper_open = [0.05]
	gripper_close = [0.5]


	gripper_traj_open = JointTrajectory()
        gripper_traj_open.joint_names = gripper_joints
        gripper_traj_open.points.append(JointTrajectoryPoint())
        gripper_traj_open.points[0].positions = gripper_open
        gripper_traj_open.points[0].velocities = [0.0 for i in gripper_joints]
        gripper_traj_open.points[0].accelerations = [0.0 for i in gripper_joints]
        gripper_traj_open.points[0].time_from_start = rospy.Duration(3.0)

	gripper_traj_close = JointTrajectory()
        gripper_traj_close.joint_names = gripper_joints
        gripper_traj_close.points.append(JointTrajectoryPoint())
        gripper_traj_close.points[0].positions = gripper_close
        gripper_traj_close.points[0].velocities = [0.0 for i in gripper_joints]
        gripper_traj_close.points[0].accelerations = [0.0 for i in gripper_joints]
        gripper_traj_close.points[0].time_from_start = rospy.Duration(3.0)


	gripper_goal = FollowJointTrajectoryGoal()
	gripper_goal.trajectory = gripper_traj_open
	gripper_goal.goal_time_tolerance = rospy.Duration(0.0)
	gripper_client.send_goal(gripper_goal)
        gripper_client.wait_for_result(rospy.Duration(5.0))

	# other setting
	shift_value = -0.05
        
        while(True):
		arm.set_named_target('home')
		arm.go()
		rospy.sleep(2)

		# arm pose1
		arm.set_start_state_to_current_state()

		arm.set_pose_target(target_pose1, end_effector_link)

		traj = arm.plan()
		arm.execute(traj)
		rospy.sleep(1)


		# move close to target
		arm.shift_pose_target(2, shift_value, end_effector_link)
		arm.go()
		rospy.sleep(1)


		# close griper
		gripper_goal = FollowJointTrajectoryGoal()
		gripper_goal.trajectory = gripper_traj_close
		gripper_goal.goal_time_tolerance = rospy.Duration(0.0)
	        gripper_client.send_goal(gripper_goal)
        	gripper_client.wait_for_result(rospy.Duration(5.0))


		
		# arm pose2
		arm.set_start_state_to_current_state()
	
		arm.set_pose_target(target_pose2, end_effector_link)
		
		traj = arm.plan()
		arm.execute(traj)
		rospy.sleep(1)


		# move close to target
		arm.shift_pose_target(2, shift_value, end_effector_link)
		arm.go()
		rospy.sleep(1)


		# open griper
		gripper_goal = FollowJointTrajectoryGoal()
		gripper_goal.trajectory = gripper_traj_open
		gripper_goal.goal_time_tolerance = rospy.Duration(0.0)
	        gripper_client.send_goal(gripper_goal)
        	gripper_client.wait_for_result(rospy.Duration(5.0))


		arm.set_named_target('home')
		arm.go()
		rospy.sleep(2)
         
        """
        arm.shift_pose_target(1, -0.05, end_effector_link)
        arm.go()
        rospy.sleep(1)
  
        
        arm.shift_pose_target(3, -1.57, end_effector_link)
        arm.go()
        rospy.sleep(1)
           
        
        arm.set_named_target('home')
        arm.go()

        """
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItIkDemo()

    
    
