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
#from geometry_msgs.msg import PoseStamped
#from sensor_msgs import JointState

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True) 

#แกกกก... ชั้นว่าชั้นบรรลุแล้วเว่ย ชั้นว่าแกใช้ชื่ออื่นได้เว่ย มันคือชื่อของโหนดที่เราจะ process ดังนั้น ไฟล์นี้จะสามารถเปลี่ยนชื่อเป็นโหนดที่เราจะ subscribe ค่ามาได้เลย ลองไปเทสดูนะ เพราะใน1ไฟล์ จะมีได้แค่1โหนดเท่านั้น เพราะฉะนั้นไฟล์นี้จะถูกใช้เป็นไฟล์ที่รับค่าจากdevice จะประมวลผลโดยการสั่งให้มันมูฟตาม โดยใช้คำสั่ง group.go(joint_goal, wait=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "ur_5"
group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

#มีการพับบลิคไปเก็บค่า plan path ไว้ที่ topic /move_group/display_planned_path เพราะงั้นลองไป echo ค่าจาก topic นั้นมาดูซิ พอเช็ค info แล้ว subscriber ของมันคือ rviz ตัวที่ใช้โชว์ robot simulation ดังนั้นคิดว่าอันนี้ไม่เกี่ยวอะไร ปล่อยมันไปแบบนี้แหละ

# We can get the name of the reference frame for this robot:
planning_frame = group.get_planning_frame()
print ("============ Reference frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
print ("============ End effector: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print ("============ Robot Groups:", robot.get_group_names())

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
#joint_goal[1] = -pi/4
#joint_goal[2] = 0
#joint_goal[3] = -pi/2
#joint_goal[4] = 0
#joint_goal[5] = pi/3

#class Mapping:
#	def __init__(self):
#		self.sub = rospy.Subscriber('/phantom/pose', PoseStamped, queue_size=1)
#		self.pub = rospy.Publisher('')
	
#if __name__ == '__main__':
#	try:
#		convert = Mapping()
#		rospy.spin()
#	except rospy.RosInterruptException:
#		pass	

#group.go(joint_goal, wait=True)
#group.stop()

pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = 1.0
pose_goal.position.x = 0.4
pose_goal.position.y = 0.0
pose_goal.position.z = 1.3
print("pose goal = ", pose_goal)
group.set_pose_target(pose_goal)
plan = group.go(wait=True)
group.stop()
group.clear_pose_targets()

#แต่คิดว่าจะต้องมีปัญหาตอนเอาค่า pose ของอุปกรณ์เข้ามาแน่ๆ เพราะมันจะเข้ามาเป็น matrix คงต้องรับค่าเข้ามาแบบ 1-by-1

