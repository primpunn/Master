#!/usr/bin/python
#
# Send joint values to UR5 using messages
#
import time
from trajectory_msgs.msg import JointTrajectory #from line 6-10 is value for publishing node
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Float32MultiArray
#from geometry_msgs.msg import Twist,Vector3
#from geometry_msgs.msg import PoseStamped 
#from geometry_msgs.msg import Point
import rospy
import actionlib
import sys, select, termios, tty 
from sensor_msgs.msg import JointState #geomagic & ur5
#reload(sys) 
#importlib(sys)
#sys.setdefaultencoding("UTF-8")
#import sys
# Set the default encoding to "UTF-8"
#if sys.getdefaultencoding() != 'utf-8':
#    reload(sys)
#    sys.setdefaultencoding('utf-8')
#from ur_kinematics import Kinematics
import math



def Call():
    
    rospy.init_node('Joint_controller', anonymous=True)
    # setup Phantom Omni joint states
    rospy.Subscriber('/phantom/joint_states', JointState, Phantom, queue_size=1) #geomagic
    rospy.Subscriber('/joint_states', JointState, main, queue_size=1) #ur5
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
# callback_State() returns 'current_state' global variable

def Phantom(state): #geomagic
    global sub
    sub = state.position #only position of joints from geomagic


def main(data): #ur5

    global pose
#    pub = rospy.Publisher('/joint_states/command',
#                          JointTrajectory,
#                          queue_size=1)
    pub = rospy.Publisher('/desired_joint_position',
                          JointTrajectory,
                          queue_size=1)                    
    ur_status = data
    pose_current = ur_status.position #only position of ur5
    
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
    Q0 = [sub[0], -sub[1], -sub[2]+1.8, 2.8+sub[2]+sub[1], -1.57, sub[5] ]

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

