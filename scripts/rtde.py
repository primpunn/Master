#!/usr/bin/env python

import rospy
import numpy as np
import rtde_receive
import rtde_control

from rtde_control import RTDEControlInterface as RTDEControl
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
#from geometry_msgs.msg import PoseStamped

class OmniStateToTwist:

    def __init__(self):
        rospy.init_node('korean_teleoperation', anonymous=True)
        self.rtde_frequency = 250
        self.rtde_c = RTDEControl("192.168.0.2", self.rtde_frequency, RTDEControl.FLAG_USE_EXT_UR_CAP)
        self.rtde_r = rtde_receive.RTDEReceiveInterface("192.168.0.2")
        #self.rtde_p = rtde_control.RTDEControlInterface("192.168.0.2")
        self.geomagic_joint_sub = rospy.Subscriber('/phantom/joint_states', JointState, self.Geomagic, queue_size=10)
        #self.geomagic_joint_sub = rospy.Subscriber('/phantom/pose', PoseStamped, self.Geomagic, queue_size=1)
        #self.ur5_joint_sub = rospy.Subscriber('/joint_states', JointState, self.UR5, queue_size=1)

        rospy.spin()

    def Geomagic (self, state):
        # vel = 0.5
        # acc = 0.5
        # dt = 1/500
        # l_time = 0.1
        # gain = 300
        # joint_speed = [0, 0, 0, 0, 0, 0]

        sub = state.position #position data from geomagic touch device
        #sub = state.pose #position data from geomagic touch device
        print("Geomagic = ", sub)
        print("\n")
        #print(sub.position.x)
        # Q0 = []
        # Q0.append(sub.position.x)
        # Q0.append(sub.position.y)
        # Q0.append(sub.position.z)
        # Q0.append(sub.orientation.x)
        # Q0.append(sub.orientation.y)
        # Q0.append(sub.orientation.z)

        # print ("Array of Geomagic's pose = \n", Q0)
        # print ("\n")
        Q0 = [sub[0], -sub[1], -sub[2]+1.8, 2.8+sub[2]+sub[1], -1.57, sub[5]] #transform and make it synchronize to geomagic touch device
        #print("After calculate = ", Q0)
        header = Header()
        header.stamp = rospy.Time.now()
        # ee_pose = self.calculate_end_effector_pose(Q0)
        # self.rtde_c.moveL(ee_pose, 0.5, 0.3)
        #self.rtde_c.moveL(Q0, 0.5, 0.3)
        self.rtde_c.moveJ(Q0, 3, 2.8)
        self.actual_q = self.rtde_r.getActualQ()
        print("ur5 = ", self.actual_q)
        print("\n")

        # for i in range(1000):
        #     self.t_start = self.rtde_c.initPeriod()
        #     self.rtde_c.speedJ(joint_speed, acc, dt)
        #     self.rtde_c.waitPeriod(self.t_start)

        # self.rtde_c.servoStop()
        # self.rtde_c.stopScript()


    # def UR5 (data):
    #     ur5 = data
    #     print("UR5 = ", ur5)
    #     print("\n")

    # def calculate_end_effector_pose(self, joint_positions):
    #     transformation_matrix = np.eye(4)
    #     transformation_matrix[0, 3] = 0.5
    #     transformation_matrix[1, 3] = 0.5
    #     transformation_matrix[2, 3] = 0.5

    #     ee_pose = np.dot(transformation_matrix, np.array([joint_positions[0], joint_positions[1], joint_positions[2], 1]))
    #     print("Change to pose = ", ee_pose)

    #     return ee_pose.tolist()

if __name__ == "__main__":
    try:
        to_twist = OmniStateToTwist()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion")
