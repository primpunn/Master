#! /usr/bin/env python

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import actionlib
import rospy
import math
import tf
import sys
import numpy
from tf.transformations import euler_from_quaternion
from tf.transformations import compose_matrix 
from tf.transformations import is_same_transform
from geometry_msgs.msg import Twist,Vector3
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_msgs.msg import Header
from moveit_commander.conversions import pose_to_list
from urdf_parser_py.urdf import URDF

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('modify_teleoperation', anonymous=True) 
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "ur_5"
group = moveit_commander.MoveGroupCommander(group_name)

#eef_link = group.get_end_effector_link()
#print ("============ End effector: %s" % eef_link)

pose_sub = rospy.Subscriber('/phantom/pose', geometry_msgs.msg.PoseStamped, queue_size=10)
joint_state_sub = rospy.Subscriber('/phantom/joint_states', sensor_msgs.msg.JointState, queue_size=10)
#state_sub = rospy.Subscriber('/phantom/state', geometry_msgs.msg.PoseStamped, queue_size=10)

print("Subscriber = ", joint_state_sub)

#pose_goal = geometry_msgs.msg.Pose()
#pose_goal = geometry_msgs.msg.Point()
#pose_goal = geometry_msgs.msg.PoseStamped()
pose_goal = sensor_msgs.msg.JointState()
#pose_goal = joint_state_sub
#pose_goal = geometry_msgs.msg.Quaternion()
#goal = float(pose_goal)
print("Pose goal = ", pose_goal)
#rospy.spin()
#group.set_pose_target(pose_goal, )
#group.set_pose_target = pose_goal
#print("Group.pose_goal = ", group.set_pose_target)
#print("Pose target = ", group.set_pose_target(pose_goal))
#print("======== group go =", group.go)
plan = group.go(wait=True)
group.stop()
group.clear_pose_targets()
