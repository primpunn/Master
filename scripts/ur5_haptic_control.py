#! /usr/bin/env python

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import actionlib
import rospy, sys
import math
import tf
import sys
import numpy
from tf.transformations import euler_from_quaternion
from tf.transformations import compose_matrix 
from tf.transformations import is_same_transform
from geometry_msgs.msg import Twist,Vector3
from geometry_msgs.msg import PoseStamped #position of geomagic
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState #position of ur5 simulation
from std_msgs.msg import String
from std_msgs.msg import Header
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "ur_5"
group = moveit_commander.MoveGroupCommander(group_name)
	
def Call():	
	rospy.init_node('modify_teleoperation', anonymous=True) 
	#geomagic_pose_sub = rospy.Subscriber('/phantom/pose', geometry_msgs.msg.PoseStamped, Phantom, queue_size=10)
	rospy.Subscriber('/phantom/joint_states', sensor_msgs.msg.JointState, Phantom, queue_size=1)
	rospy.Subscriber('/joint_states', sensor_msgs.msg.JointState, main, queue_size=1) #position of ur5 simulation
	rospy.spin()
    	
#Phantom state
def Phantom (state):
    	global sub
    	sub = state.position
    	#print("state_position = ", sub)
    #print (sub)

#Niryo arm Control
def main(data):
	ur5_status = data
	#pose_current = ur5_status.position
	pose_current = robot.get_current_state()
	#traj=n.get_joints()
	#print("position of ur5_sim = ", pose_current)
	rate = rospy.Rate(125)
	
	while not rospy.is_shutdown():
		#Q0 = [sub[0]*2.5, sub[1]/2, sub[2]+0.36,-sub[3]+1.9, -sub[4]-2, -sub[5]/3]
		#Q0 = [sub[0], sub[1], sub[2], sub[3], sub[4], sub[5]]
        #n.move_joints(Q0)
		Q0 = [sub[0], -sub[1], -sub[2]+1.8, 2.8+sub[2]+sub[1], -1.57, sub[5] ]
        	#Q0 = [[[cos(sub[0])], 0, ]]
		pose_goal = geometry_msgs.msg.Pose()
		print("position of ur5_sim = ", pose_current)
		#print("position command for ur5 = ", Q0)
		#group.set_pose_target(Q0)
		#plan = group.go(wait=True)
		plan = group.go(Q0)
		group.stop()
		group.clear_pose_targets()
        #if greybutton:
        #    n.open_gripper(TOOL_GRIPPER_2_ID,500)
        #elif whitebutton:
        #    n.close_gripper(TOOL_GRIPPER_2_ID,500)
    #rospy.loginfo(traj)
    #rospy.loginfo(data)
		rate.sleep()


if __name__ == '__main__':
    try:
        Call()
        #Phantom
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
