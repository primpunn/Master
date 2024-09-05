#!/usr/bin/python
import rospy
#from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionGoal


def joint_position_callback(msg):
    # Extract the joint positions from the received message
    joint_positions = msg.points[0].positions
    #pts.positions = msg.data
    
    # Command the UR5 robot arm to move with the received joint positions
    #pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
    pub = rospy.Publisher('/scaled_pos_joint_traj_controller/follow_joint_trajectory/publisher', FollowJointTrajectoryActionGoal, queue_size=1) #แต่พอเอคโค่ค่าจากทอปปิกที่พลับบลิคออกไปมาดู ไม่มีค่าออกมา แปลว่าตรงบรรทัดที่ 26 ยังพับบลิคค่าขึ้นมาที่ทอปปิกนี้ไม่ได้
    
    joint_trajectory = JointTrajectory()
    joint_trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
    point = JointTrajectoryPoint()
    point.positions = joint_positions
    point.time_from_start = rospy.Duration(1.0)  # Duration to reach the joint positions

    joint_trajectory.points.append(point)
    print(point) #มีค่าออกมาแล้ว หลังจากรันไฟล์นี้ แปลว่าจนถึงตรงนี้ทำงานได้ดี
    pub.publish(point)
    
    
def joint_position_subscriber():
    rospy.init_node('joint_position_subscriber', anonymous=True)
    rospy.Subscriber('/desired_joint_position', JointTrajectory, joint_position_callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    try:
        joint_position_subscriber()
    except rospy.ROSInterruptException:
        pass
