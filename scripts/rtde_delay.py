#!/usr/bin/env python

# from rtde_control import RTDEControlInterface as RTDEControl
# from rtde_receive import RTDEReceiveInterface as RTDEReceive
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Header

# import datetime
# import math
# import os
# import psutil
# import sys
# import rospy

# # def getCircleTarget(pose, timestep, radius=0.075, freq=1.0):
# #     circ_target = pose[:]
# #     circ_target[0] = pose[0] + radius * math.cos((2 * math.pi * freq * timestep))
# #     circ_target[1] = pose[1] + radius * math.sin((2 * math.pi * freq * timestep))
# #     return circ_target
# Q1 = None

# def OmniStateToTwist(geomagic_joint_state):
#     rospy.init_node('korean_teleoperation', anonymous=True)
#     # sub0 = geomagic_joint_sub
#     # sub = sub0.position
#     # print("Geomagic = ", sub)
#     # print("\n")
#     #global Q0
#     # Q0 = [sub[0], -sub[1], -sub[2]+1.8, 2.8+sub[2]+sub[1], -1.57, sub[5]] #transform and make it synchronize to geomagic touch device
#     q0 = geomagic_joint_state.posiiton[0]
#     q1 = geomagic_joint_state.position[1]
#     q2 = geomagic_joint_state.position[2]
#     q3 = geomagic_joint_state.position[3]
#     q4 = geomagic_joint_state.position[4]
#     q5 = geomagic_joint_state.position[5]
#     return [q0, -q1, -q2+1.8, 2.8+q2+q1, -1.57, q5]

# def callback(geomagic_joint_state):
# # def Geomagic():
# #     # global sub
# #     sub = geomagic_joint_sub.position
# #     return sub
# # Q1 = Geomagic(state="positon")
#     global Q1
#     Q1 = OmniStateToTwist(geomagic_joint_state) #transform and make it synchronize to geomagic touch device
# # Q0 = [sub[0], -sub[1], -sub[2]+1.8, 2.8+sub[2]+sub[1], -1.57, sub[5]] #transform and make it synchronize to geomagic touch device
#     print(Q1)

# geomagic_joint_sub = rospy.Subscriber('/phantom/joint_states', JointState, callback, queue_size=10)
# while Q1 is None:
#     pass

# vel = 0.5
# acc = 0.5
# rtde_frequency = 500.0
# dt = 1.0/rtde_frequency  # 2ms
# flags = RTDEControl.FLAG_VERBOSE | RTDEControl.FLAG_UPLOAD_SCRIPT
# ur_cap_port = 50002
# robot_ip = "192.168.0.2"

# lookahead_time = 0.1
# gain = 600

# # ur_rtde realtime priorities
# rt_receive_priority = 90
# rt_control_priority = 85
# #geomagic_joint_sub = rospy.Subscriber('/phantom/joint_states', JointState, queue_size=10)
# rtde_r = RTDEReceive(robot_ip, rtde_frequency, [], True, False, rt_receive_priority)
# rtde_c = RTDEControl(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority)

# # header = Header()
# # header.stamp = rospy.Time.now()
# # Set application real-time priority
# os_used = sys.platform
# process = psutil.Process(os.getpid())
# if os_used == "win32":  # Windows (either 32-bit or 64-bit)
#     process.nice(psutil.REALTIME_PRIORITY_CLASS)
# elif os_used == "linux":  # linux
#     rt_app_priority = 80
#     param = os.sched_param(rt_app_priority)
#     try:
#         os.sched_setscheduler(0, os.SCHED_FIFO, param)
#     except OSError:
#         print("Failed to set real-time process scheduler to %u, priority %u" % (os.SCHED_FIFO, rt_app_priority))
#     else:
#         print("Process real-time priority set to: %u" % rt_app_priority)

# time_counter = 0.0

# # Move to init position using moveL
# actual_tcp_pose = rtde_r.getActualTCPPose()
# #init_pose = getCircleTarget(actual_tcp_pose, time_counter)
# init_pose = Q1
# #self.rtde_c.moveL(init_pose, self.vel, self.acc)
# rtde_c.moveJ(Q1, vel, acc)

# try:
#     while True:
#         t_start = rtde_c.initPeriod()
#         #servo_target = getCircleTarget(actual_tcp_pose, time_counter)
#         servo_target = OmniStateToTwist(actual_tcp_pose, time_counter)
#         #rtde_c.servoL(servo_target, vel, acc, dt, lookahead_time, gain)
#         rtde_c.servoJ(servo_target, vel, acc, dt, lookahead_time, gain)
#         rtde_c.waitPeriod(t_start)
#         time_counter += dt

# except KeyboardInterrupt:
#     print("Control Interrupted!")
#     rtde_c.servoStop()
#     rtde_c.stopScript()

# import rospy
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Header
# from rtde_control import RTDEControlInterface as RTDEControl
# from rtde_receive import RTDEReceiveInterface as RTDEReceive
# import psutil
# import os
# import sys
# import matplotlib.pyplot as plt

# # Global variable to store the latest joint states
# current_joint_state = None
# ur5_joint_state = None

# def joint_state_callback(msg):
#     global current_joint_state
#     current_joint_state = msg.position
#     print("Geomagic = ", current_joint_state)

# def initialize_robot_connection(robot_ip, rtde_frequency, rt_receive_priority, rt_control_priority):
#     flags = RTDEControl.FLAG_VERBOSE | RTDEControl.FLAG_UPLOAD_SCRIPT
#     ur_cap_port = 50002
    
#     rtde_r = RTDEReceive(robot_ip, rtde_frequency, [], True, False, rt_receive_priority)
#     rtde_c = RTDEControl(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority)
    
#     return rtde_r, rtde_c

# def set_realtime_priority():
#     os_used = sys.platform
#     process = psutil.Process(os.getpid())
#     if os_used == "win32":  # Windows
#         process.nice(psutil.REALTIME_PRIORITY_CLASS)
#     elif os_used == "linux":  # Linux
#         rt_app_priority = 80
#         param = os.sched_param(rt_app_priority)
#         try:
#             os.sched_setscheduler(0, os.SCHED_FIFO, param)
#         except OSError:
#             rospy.logerr("Failed to set real-time process scheduler to %u, priority %u" % (os.SCHED_FIFO, rt_app_priority))
#         else:
#             rospy.loginfo("Process real-time priority set to: %u" % rt_app_priority)

# class DataCollector:
#     def __init__(self):
#         self.noisy_data = []
#         rospy.init_node('korean_teleoperation', anonymous=True)
#         self.geomagic_joint_sub = rospy.Subscriber('/phantom/joint_states', JointState, self.joint_state_callback, queue_size=1)
        
#         robot_ip = "192.168.0.2"
#         rtde_frequency = 500.0
#         rt_receive_priority = 500
#         rt_control_priority = 500
        
#         rtde_r, rtde_c = initialize_robot_connection(robot_ip, rtde_frequency, rt_receive_priority, rt_control_priority)
#         set_realtime_priority()

#         vel = 3.14
#         acc = 3.14
#         self.dt = 0.1 / rtde_frequency
#         self.lookahead_time = 0.1
#         self.gain = 1000

#         try:
#             while not rospy.is_shutdown():
#                 if current_joint_state is not None:
#                     Q0 = [current_joint_state[0], -current_joint_state[1], -current_joint_state[2]+1.8, 2.8+current_joint_state[2]+current_joint_state[1], -1.57, current_joint_state[5]]
#                     rtde_c.moveJ(Q0, vel, acc)
#                     actual_q = rtde_r.getActualQ()
#                     print("ur5 = ", actual_q)
#                     print("\n")
#                     rospy.sleep(0.1)  # Sleep to throttle the loop
#                 else:
#                     rospy.logwarn("Waiting for joint state updates...")
#                     rospy.sleep(0.1)
                    
#         except KeyboardInterrupt:
#             rospy.loginfo("Control interrupted!")
#         finally:
#             rtde_c.servoStop()
#             rtde_c.stopScript()

#     def get_data(self):
#         rospy.spin()
#         return self.noisy_data

# if __name__ == '__main__':
#     #main()
#     collector = DataCollector()
#     print("Collecting data, press Ctrl+C to stop...")

#     try:
#         collector.get_data()
#     except KeyboardInterrupt:
#         pass

#     noisy_data = collector.noisy_data
#     if not noisy_data:
#         print("No data collected.")
#         exit()
    
#     process_variance = 1e-5
#     measurement_variance = 1
#     initial_value = noisy_data[0]

#     plt.figure(figsize=(10, 6))
#     plt.plot(noisy_data, label='Noisy Data')



# def main():
#     rospy.init_node('korean_teleoperation', anonymous=True)
#     geomagic_joint_sub = rospy.Subscriber('/phantom/joint_states', JointState, joint_state_callback, queue_size=1)
    
#     robot_ip = "192.168.0.2"
#     rtde_frequency = 500.0
#     rt_receive_priority = 500
#     rt_control_priority = 500
    
#     rtde_r, rtde_c = initialize_robot_connection(robot_ip, rtde_frequency, rt_receive_priority, rt_control_priority)
#     set_realtime_priority()

#     vel = 3.14
#     acc = 3.14
#     dt = 0.1 / rtde_frequency
#     lookahead_time = 0.1
#     gain = 1000

#     try:
#         while not rospy.is_shutdown():
#             if current_joint_state is not None:
#                 Q0 = [current_joint_state[0], -current_joint_state[1], -current_joint_state[2]+1.8, 2.8+current_joint_state[2]+current_joint_state[1], -1.57, current_joint_state[5]]
#                 rtde_c.moveJ(Q0, vel, acc)
#                 rospy.sleep(0.1)  # Sleep to throttle the loop
#             else:
#                 rospy.logwarn("Waiting for joint state updates...")
#                 rospy.sleep(0.1)
                
#     except KeyboardInterrupt:
#         rospy.loginfo("Control interrupted!")
#     finally:
#         rtde_c.servoStop()
#         rtde_c.stopScript()

import rospy
import rtde_control
import rtde_receive
import threading
import time
import psutil
import os
import sys
import matplotlib.pyplot as plt
from sensor_msgs.msg import JointState
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

# Global variable to store the latest joint states
global current_joint_state
current_joint_state = None

ur5_joint_state = None

def joint_state_callback(data):
    global current_joint_state
    current_joint_state = data.position
    # print("Geomagic = ", current_joint_state) #position of geomagic
    return current_joint_state

def initialize_robot_connection(robot_ip, rtde_frequency, rt_receive_priority, rt_control_priority):
    flags = RTDEControl.FLAG_VERBOSE | RTDEControl.FLAG_UPLOAD_SCRIPT
    ur_cap_port = 50002
    
    rtde_r = RTDEReceive(robot_ip, rtde_frequency, [], True, False, rt_receive_priority)
    rtde_c = RTDEControl(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority)
    
    # Make sure to return the created interfaces
    return rtde_r, rtde_c

def set_realtime_priority():
    # ... existing code ...
    os_used = sys.platform
    process = psutil.Process(os.getpid())
    if os_used == "win32":  # Windows
        process.nice(psutil.REALTIME_PRIORITY_CLASS)
    elif os_used == "linux":  # Linux
        rt_app_priority = 80
        param = os.sched_param(rt_app_priority)
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, param)
        except OSError:
            rospy.logerr("Failed to set real-time process scheduler to %u, priority %u" % (os.SCHED_FIFO, rt_app_priority))
        else:
            rospy.loginfo("Process real-time priority set to: %u" % rt_app_priority)

class DataCollector:
    def __init__(self):
        global ur5_joint_state
        self.geomagic_data = []
        self.ur5_data = []
        rospy.init_node('korean_teleoperation', anonymous=True)
        self.geomagic_joint_sub = rospy.Subscriber('/phantom/joint_states', JointState, joint_state_callback, queue_size=1)
        rate = rospy.Rate(50)
        # rate = rospy.Rate(rospy.get_param('~hz', 50))
        print("Subscription Rate:"+str(rate))
        
        # ... existing code for robot_ip, rtde_frequency, rt_receive_priority, rt_control_priority ...
        robot_ip = "192.168.1.2"
        rtde_frequency = 500
        rt_receive_priority = 500
        rt_control_priority = 500
    
#     rtde_r, rtde_c = initialize_robot_connection(robot_ip, rtde_frequency, rt_receive_priority, rt_control_priority)
#     set_realtime_priority()


        rtde_r, rtde_c = initialize_robot_connection(robot_ip, rtde_frequency, rt_receive_priority, rt_control_priority)
        set_realtime_priority()
        # self.rtde_receive = rtde_receive.RTDEReceiveInterface(robot_ip)
        # self.rtde_control = rtde_control.RTDEControlInterface(robot_ip)
        self.running = True
        self.data_lock = threading.Lock()
        self.latest_data = None
        

        # ... existing code for vel, acc, etc. ...
        vel = 3.14
        acc = 0.5
        dt = 1.0 / rtde_frequency
        lookahead_time = 0.1
        gain = 600
        
        try:
            while not rospy.is_shutdown():
                if current_joint_state is not None:
                    # Here, we're saving the current joint state of the Geomagic device
                    # self.geomagic_data.append(current_joint_state)
                    print("Geomagic = ", current_joint_state)
                    # Q0 = [current_joint_state[0], current_joint_state[1], current_joint_state[2], current_joint_state[3], current_joint_state[4], current_joint_state[5]]                   
                    Q0 = [current_joint_state[0], (-1)*current_joint_state[1], -current_joint_state[2]+1.8, 2.8+current_joint_state[2]+current_joint_state[1], -1.57, 0]
                    # Q0 = [current_joint_state[0], (-1)*current_joint_state[1], -current_joint_state[2], current_joint_state[2]+current_joint_state[1]+2.8, -1.57, 0]
                    # เพื่อล็อค4ไว้ ไม่ว่า 2, 3 จะหมุนเปลี่ยนยังไง แต่4 ยังจะล็อคไว้ที่ 90 เหมือนเดิม
                    # self.geomagic_data.append(Q0)  
                    self.geomagic_data.append(current_joint_state)                   
                    rtde_c.servoJ(Q0, vel, acc,dt,lookahead_time,gain)
                    print(time.time)
                    actual_q = rtde_r.getActualQ()
                    
                    # Here, we're saving the current joint state of the UR5 robot
                    self.ur5_data.append(actual_q)
                    ur5_joint_state = actual_q
                    
                    print("ur5 = ", actual_q)
                    print("\n")
                    rospy.sleep(1/rtde_frequency)  # Sleep to throttle the loop
                else:
                    rospy.logwarn("Waiting for joint state updates...")
                    rospy.sleep(0.1)
                    
        except KeyboardInterrupt:
            rospy.loginfo("Control interrupted!")
        finally:
            rtde_c.servoStop()
            rtde_c.stopScript()

    def start_receiving(self):
        threading.Thread(target=self.receive_loop).start()

    def receive_loop(self):
        while self.running:
            start_time = time.time()
            data = self.rtde_receive.getActualQ()  # Replace with the actual data you need
            with self.data_lock:
                self.latest_data = data
            elapsed_time = time.time() - start_time
            time.sleep(max(0, 1.0 / 500 - elapsed_time))  # Target 500Hz frequency

    def get_latest_data(self):
        with self.data_lock:
            return

    def plot_data(self):
        # Ensure that we have collected data before attempting to plot
        if not self.geomagic_data or not self.ur5_data:
            print("No data to plot.")
            return
        
        # Convert the collected data into a format suitable for plotting
        geomagic_data_to_plot = list(zip(*self.geomagic_data))
        ur5_data_to_plot = list(zip(*self.ur5_data))
        
        # Create subplots for each joint
        plt.figure(figsize=(15, 10))
        for i in range(6):
            plt.subplot(3, 2, i+1)
            plt.plot(geomagic_data_to_plot[i], label='Geomagic Joint {}'.format(i+1))
            plt.plot(ur5_data_to_plot[i], label='UR5 Joint {}'.format(i+1))
            plt.xlabel('Timestamp')
            plt.ylabel('Position (rad)')
            plt.title('Joint {}'.format(i+1))
            plt.legend()
        
        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    collector = DataCollector()
    print("Collecting data, press Ctrl+C to stop ...")
    try:
        # collector = DataCollector()
        # print("Collecting data, press Ctrl+C to stop...")
        rospy.spin()
    except KeyboardInterrupt:
        # rospy.loginfo("KeyboardInterrupt received, shutting down.")
        pass
    # except Exception as e:
    #     rospy.logerr("An error occured: %s" % str(e))
    # finally:
    #     if collector.rtde_control is not None:
    #         collector.rtde_control.servoStop()
    #         collector.rtde_control.stopScript()
    #     if collector.rtde_receive is not None:
    #         collector.rtde_receive.disconnect()        
    collector.plot_data()