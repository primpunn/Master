#!/usr/bin/python
#
# Send joint values to UR5 using messages
#
import moveit_commander
import moveit_msgs.msg
import sensor_msgs.msg
import time
import rospy
import actionlib
import sys, select, termios, tty 
import math
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped #geomagic
from sensor_msgs.msg import JointState #ur5
from moveit_commander.conversions import pose_to_list

#reload(sys) 
#sys.setdefaultencoding("UTF-8")
#from ur_kinematics import Kinematics

moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()

def Call():  
    rospy.init_node('send_ur5', anonymous=True)
    # setup Phantom Omni joint states
    rospy.Subscriber('/phantom/joint_states', sensor_msgs.msg.JointState, Phantom, queue_size=1)
    rospy.Subscriber('/joint_states', sensor_msgs.msg.JointState, main, queue_size=1)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
# callback_State() returns 'current_state' global variable

def Phantom(state):
    global sub
    sub = state.position

#def ur_get_status():
#    current_data = rospy.wait_for_message("arm_controller/state", JointTrajectoryControllerState)
#    return current_data
#Q0 = [-0.12694727059672406, -1.331667696607827, 2.391941365528808, -1.1109140138393911, 1.545242764007165, 0.13237981553654432]

def main(data): 	
    global pose
    pub = rospy.Publisher('arm_controller/command',
                          JointTrajectory,
                          queue_size=1)
    ur_status = data
    #pose_current = ur_status.actual.positions
    pose_current = robot.get_current_state()
    
    # Create the topic message
    traj = JointTrajectory()
    traj.header = Header()
    # Joint names for UR5
    traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                        'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                        'wrist_3_joint']

    rate = rospy.Rate(125) # 125hz = 0.08s
    
    traj.header.stamp = rospy.Time.now()
    pts = JointTrajectoryPoint()


    user_value = -0.15

    # six joints values array
    Q0 = [sub[0], -sub[1], -sub[2]+1.8, 2.8+sub[2]+sub[1], -1.57, sub[5] ] #[sub[0], -231667696607827, 2.391941365528808,
         # -1.1109140138391, 1.545242764007165, #0.13237981553654432]


    pts.positions = Q0
    pts.time_from_start = rospy.Duration(0.1)
    # Set the points to the trajectory
    traj.points = []
    traj.points.append(pts)
    # Publish the message
    rospy.loginfo(traj)
    rospy.loginfo(pose_current)
    pub.publish(traj)
    rate.sleep()


if __name__ == '__main__':
    try:
        Call()
        #Phantom
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
