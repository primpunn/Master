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
from geometry_msgs.msg import PoseStamped
#from sensor_msgs.msg import JointState

def callback(msg):
	#rospy.loginfo(rospy.get_caller_id() + "I heard %s", geometry_msgs.msg)
	rospy.loginfo(rospy.get_caller_id())
	
def receive_message():
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('teleoperation', anonymous=True)
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	joint_goal = rospy.Subscriber("/phantom/pose", PoseStamped, callback)
	group_name = "ur_5"
	group = moveit_commander.MoveGroupCommander(group_name)

	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

# We can get the name of the reference frame for this robot:
	planning_frame = group.get_planning_frame()
	#print ("============ Reference frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
	eef_link = group.get_end_effector_link()
	#print ("============ End effector: %s" % eef_link)

# We can get a list of all the groups in the robot:
	group_names = robot.get_group_names()
	#print ("============ Robot Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
	print ("============ Printing robot state")
	print (robot.get_current_state())
	print ("")

#joint_goal = group.get_current_joint_values()
#shoulder_pan_joint = 0
#shoulder_lift_joint = -pi/4
#elbow_joint = 0
#wrist_1_joint = -pi/2
#wrist_2_joint = 0
#wrist_3_joint = pi/3

#joint_goal[0] = 0
#joint_goal[1] = 0
#joint_goal[2] = 0
#joint_goal[3] = -pi/2
#joint_goal[4] = 0
#joint_goal[5] = pi/3

if __name__ == '__main__':
	receive_message()
	group_name = "ur_5"
	group = moveit_commander.MoveGroupCommander(group_name)
	#group.go(joint_goal, wait=True)
	#group.stop()
	
	pose_goal = geometry_msgs.msg.Pose()
	group.set_pose_target(pose_goal)
	plan = group.go(wait=True)
	group.stop()
	group.clear_pose_targets()

