#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
# from iiwa_msgs.msg import MoveToJointPositionAction, MoveToJointPositionGoal # iiwa_msgs is a package name;MoveToJointPositionAction is a .msg file
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryDemo():
    def __init__(self):
        rospy.init_node('trajectory_demo')
        
        reset = rospy.get_param('~reset', False)
        """
        arm_joints = ['iiwa_joint_1',
                      'iiwa_joint_2',
                      'iiwa_joint_3', 
                      'iiwa_joint_4',
                      'iiwa_joint_5',
                      'iiwa_joint_6',
                      'iiwa_joint_7']
        """

	arm_joints = ['finger_joint']

	"""
        if reset:
            arm_goal  = [0, 0, 0, 0, 0, 0, 0]

        else:
            arm_goal  = [-0.3, -1.0, 0, 0.8, 0, 0, 0]
        """

	arm_goal = [0.5]
	
        rospy.loginfo('Waiting for arm trajectory controller...')       
        arm_client = actionlib.SimpleActionClient('iiwa/gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        # arm_client = actionlib.SimpleActionClient('iiwa/PositionJointInterface_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        arm_client.wait_for_server()        
        rospy.loginfo('...connected.')  
    

        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = arm_joints
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[0].positions = arm_goal
        arm_trajectory.points[0].velocities = [0.0 for i in arm_joints]
        arm_trajectory.points[0].accelerations = [0.0 for i in arm_joints]
        arm_trajectory.points[0].time_from_start = rospy.Duration(3.0)
    
        rospy.loginfo('Moving the arm to goal position...')
        

        arm_goal = FollowJointTrajectoryGoal()
        

        arm_goal.trajectory = arm_trajectory
        

        arm_goal.goal_time_tolerance = rospy.Duration(0.0)
    

        arm_client.send_goal(arm_goal)


        arm_client.wait_for_result(rospy.Duration(5.0))
        
        rospy.loginfo('...done')
        
if __name__ == '__main__':
    try:
        TrajectoryDemo()
    except rospy.ROSInterruptException:
        pass
    
