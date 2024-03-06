#! /usr/bin/env python

#Include the necessary libraries 
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class MyRobot:
	def __init__(self, ur_5):
		self.moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('plan_pose', anonymous=True)

		self.robot = moveit_commander.RobotCommander()
		self.scence = moveit_commander.PlanningSceneInterface()
		self.planning = ur_5
		self.group = moveit_commander.MoveGroupCommander(ur5)
		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)
		
def main():

    #Create a new arm object from the MyRobot class
    arm = MyRobot("ur_5")
    #hand =  MyRobot("hand")

    #Here, we will repeat the cycle of setting to various positions, simulating the pick and place action
    while not rospy.is_shutdown():
        #call the function to set the position to "zero_pose"
      
        arm.set_pose("start")
        rospy.sleep(2)
        
        #Open the gripper or end effector
        hand.set_pose("turn")
        rospy.sleep(1)
        
        arm.set_pose("another_side")
        rospy.sleep(2)
        #Close the gripper or end effector
        hand.set_pose("hand_closed")
        rospy.sleep(1)
        
        arm.set_pose("move_ankle")
        rospy.sleep(2)

#pose_target = geometry_msgs.msg.Pose()
#pose_target.orientation.w = 1.0
#pose_target.position.x = 0.96
#pose_target.position.y = 0
#pose_target.position.z = 1.18

#plan1 = group.plan()
#rospy.sleep(5)
moveit_commander.roscpp_shutdown()
