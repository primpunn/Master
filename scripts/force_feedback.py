#! /usr/bin/env python

#import moveit_commander
#import moveit_msgs.msg
import rospy
# import math
# import tf
# import sys
# import numpy
import omni_msgs.msg
import geometry_msgs.msg
import numpy as np 
import scipy.signal as signal
import matplotlib.pyplot as plt
#import sensor.msgs.msg
import actionlib
from geometry_msgs.msg import Wrench
from omni_msgs.msg import OmniFeedback
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise


# ftsensor = None
# geomagic = None

# def call():
#     global ftsensor, geomagic
#     rospy.init_node('force_feedback', anonymous=True)
#     ftsensor = rospy.Subscriber('/wrench_topic', geometry_msgs.msg.Wrench, sensor_callback, queue_size=1)
#     geomagic = rospy.Publisher('/phantom/force_feedback', omni_msgs.msg.OmniFeedback, queue_size=1)
#     rospy.spin()

# def sensor_callback(msg):
#     original = msg.force
#     print("Original force\n",original)
#     #msg.force = [False, False, False]
#     #geomagic.publish(msg.force)
#     #rospy.loginfo('force x:{}'.format(msg.force.x))
#     #rospy.loginfo('force y:{}'.format(msg.force.y))
#     #rospy.loginfo('force z:{}'.format(msg.force.z))
#     kf_x = KalmanFilter(dim_x=1, dim_z=1)
#     kf_x.x = np.array([original.x])
#     kf_x.F = np.array([[1.]])
#     kf_x.H = np.array([[1.]])
#     kf_x.P *= ([[1000]])
#     kf_x.R = ([[5]])
#     kf_x.Q = np.array([[2]])
#     # kf_x.Q = Q_discrete_white_noise(dim=1, dt=0.1, var=2.0, block_size=1)

#     kf_y = KalmanFilter(dim_x=1, dim_z=1)
#     kf_y.x = np.array([original.y])
#     kf_y.F = np.array([[1.]])
#     kf_y.H = np.array([[1.]])
#     kf_y.P *= ([[1000]])
#     kf_y.R = ([[5]])
#     kf_y.Q = np.array([[2]])
#     # kf_y.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=2.0, block_size=1)

#     kf_z = KalmanFilter(dim_x=1, dim_z=1)
#     kf_z.x = np.array([original.z])
#     kf_z.F = np.array([[1.]])
#     kf_z.H = np.array([[1.]])
#     kf_z.P *= ([[1000]])
#     kf_z.R = ([[5]])
#     kf_z.Q = np.array([[2]])
#     # kf_z.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=2.0, block_size=1)

#     kf_x.predict()
#     kf_x.update(np.array([[original.x]]))
#     filtered_force_x = kf_x.x[0]

#     kf_y.predict()
#     kf_y.update(np.array([[original.y]]))
#     filtered_force_y = kf_y.x[0]
    
#     kf_z.predict()
#     kf_z.update(np.array([[original.z]]))
#     filtered_force_z = kf_z.x[0]

#     filtered_force_msg = geometry_msgs.msg.Wrench()
#     filtered_force_msg.force.x = filtered_force_x
#     filtered_force_msg.force.y = filtered_force_y
#     filtered_force_msg.force.z = filtered_force_z

#     force_feedback_msg = omni_msgs.msg.OmniFeedback()
#     force_feedback_msg.force = filtered_force_msg.force
#     geomagic.publish(force_feedback_msg)
    
#     print("Filtered force\n", filtered_force_msg.force)

#     # filtered_force_data = []
#     # original1 = []
#     # original1.append(original.x)
#     # original1.append(original.y)
#     # original1.append(original.z)
#     # print("Append\n",original1)


#     # for z in original1:
#     #     kf.predict()
#     #     kf.update(z)
#     #     filtered_force_data.append(kf.x[0])

#     # plt.plot(original, label='Original force data')
#     # plt.plot(filtered_force_data, label='Filtered force data')
#     # plt.xlabel('Time')
#     # plt.ylabel('Force')
#     # plt.legend()
#     # plt.show()

#     # median_filter = signal.medfilt(original,kernel_size=1)

#     # print("Filter force\n",median_filter)
#     # force_feedback_msg = omni_msgs.msg.OmniFeedback()
#     # force_feedback_msg.force = msg.force
#     # geomagic.publish(force_feedback_msg)
#     # print("\n")
    
# if __name__ == '__main__':
#     try:
#         call()
#     except rospy.ROSInterruptException:
#         print("Program interrupted before complrtion")



# --------------------------------------------------------------------------------------------------------


# ftsensor = None
# geomagic = None

# def call():
#     global ftsensor, geomagic
#     rospy.init_node('force_feedback', anonymous=True)
#     ftsensor = rospy.Subscriber('/wrench_topic', geometry_msgs.msg.Wrench, sensor_callback, queue_size=1)
#     geomagic = rospy.Publisher('/phantom/force_feedback', omni_msgs.msg.OmniFeedback, queue_size=1)
#     rospy.spin()

# def sensor_callback(msg):
#     original = msg.force
#     print("Original force\n",original)

#     kf_x = KalmanFilter(dim_x=1, dim_z=1)
#     kf_x.x = np.array([original.x])
#     kf_x.F = np.array([[1.]])
#     kf_x.H = np.array([[1.]])
#     kf_x.P *= ([[1000]])
#     kf_x.R = ([[5]])
#     kf_x.Q = np.array([[2]])

#     kf_y = KalmanFilter(dim_x=1, dim_z=1)
#     kf_y.x = np.array([original.y])
#     kf_y.F = np.array([[1.]])
#     kf_y.H = np.array([[1.]])
#     kf_y.P *= ([[1000]])
#     kf_y.R = ([[5]])
#     kf_y.Q = np.array([[2]])

#     kf_z = KalmanFilter(dim_x=1, dim_z=1)
#     kf_z.x = np.array([original.z])
#     kf_z.F = np.array([[1.]])
#     kf_z.H = np.array([[1.]])
#     kf_z.P *= ([[1000]])
#     kf_z.R = ([[5]])
#     kf_z.Q = np.array([[2]])

#     kf_x.predict()
#     kf_x.update(np.array([[original.x]]))
#     filtered_force_x = kf_x.x[0]

#     kf_y.predict()
#     kf_y.update(np.array([[original.y]]))
#     filtered_force_y = kf_y.x[0]
    
#     kf_z.predict()
#     kf_z.update(np.array([[original.z]]))
#     filtered_force_z = kf_z.x[0]

#     filtered_force_msg = geometry_msgs.msg.Wrench()
#     filtered_force_msg.force.x = filtered_force_x
#     filtered_force_msg.force.y = filtered_force_y
#     filtered_force_msg.force.z = filtered_force_z

#     force_feedback_msg = omni_msgs.msg.OmniFeedback()
#     force_feedback_msg.force = filtered_force_msg.force
#     geomagic.publish(force_feedback_msg)
    
#     print("Filtered force\n", filtered_force_msg.force)

# if __name__ == '__main__':
#     try:
#         call()
#     except rospy.ROSInterruptException:
#         print("Program interrupted before complrtion")


# ---------------------------------------------------------------------------------------------------

# ftsensor = None
# geomagic = None

# def call():
#     global ftsensor, geomagic
#     rospy.init_node('force_feedback', anonymous=True)
#     ftsensor = rospy.Subscriber('/wrench_topic', geometry_msgs.msg.Wrench, sensor_callback, queue_size=1)
#     geomagic = rospy.Publisher('/phantom/force_feedback', omni_msgs.msg.OmniFeedback, queue_size=1)
#     rospy.spin()

# def sensor_callback(msg):
#     original = msg.force
#     print("Original force\n",original)

#     kf_x = KalmanFilter(dim_x=1, dim_z=1)
#     kf_x.x = np.array([original.x])
#     kf_x.F = np.array([[1.]])
#     kf_x.H = np.array([[1.]])
#     kf_x.P *= ([[1000]])
#     kf_x.R = ([[5]])
#     kf_x.Q = np.array([[2]])

#     kf_y = KalmanFilter(dim_x=1, dim_z=1)
#     kf_y.x = np.array([original.y])
#     kf_y.F = np.array([[1.]])
#     kf_y.H = np.array([[1.]])
#     kf_y.P *= ([[1000]])
#     kf_y.R = ([[5]])
#     kf_y.Q = np.array([[2]])

#     kf_z = KalmanFilter(dim_x=1, dim_z=1)
#     kf_z.x = np.array([original.z])
#     kf_z.F = np.array([[1.]])
#     kf_z.H = np.array([[1.]])
#     kf_z.P *= ([[1000]])
#     kf_z.R = ([[5]])
#     kf_z.Q = np.array([[2]])

#     kf_x.predict()
#     kf_x.update(np.array([[original.x]]))
#     filtered_force_x = kf_x.x[0]

#     kf_y.predict()
#     kf_y.update(np.array([[original.y]]))
#     filtered_force_y = kf_y.x[0]
    
#     kf_z.predict()
#     kf_z.update(np.array([[original.z]]))
#     filtered_force_z = kf_z.x[0]

#     filtered_force_msg = geometry_msgs.msg.Wrench()
#     filtered_force_msg.force.x = filtered_force_x
#     filtered_force_msg.force.y = filtered_force_y
#     filtered_force_msg.force.z = filtered_force_z

#     force_feedback_msg = omni_msgs.msg.OmniFeedback()
#     force_feedback_msg.force = filtered_force_msg.force
#     geomagic.publish(force_feedback_msg)
    
#     print("Filtered force\n", filtered_force_msg.force)

# class KalmanFilter:
#     def __init__(self, process_variance, measurement_variance, estimation_error, initial_value):
#         self.process_variance = process_variance
#         self.measurement_variance = measurement_variance
#         self.estimation_error = estimation_error
#         self.estimated_value = initial_value
#         self.kalman_gain = 0

#     def update(self, measurement):
#         self.estimation_error = self.estimation_error + self.process_variance
#         self.kalman_gain = self.estimation_error / (self.estimation_error + self.measurement_variance)
#         self.estimated_value = self.estimated_value + self.kalman_gain * (measurement - self.estimated_value)
#         self.estimation_error = (1 - self.kalman_gain) * self.estimation_error

#         return self.estimated_value
    
#     def apply_kalman_filter(noisy_data, process_variance, measurement_variance, initial_value):
#         kalman_filter = KalmanFilter(process_variance, measurement_variance, estimation_error=1.0, initial_value=initial_value)
#         filtered_data = []

#         for measurement in noisy_data:
#             if abs(measurement) < 10:
#                 filtered_value = kalman_filter.update(measurement)
#             else:
#                 filtered_value = measurement
#                 print("No filtering method will be applied.")
#             filtered_data.append(filtered_value)

#         return filtered_data

# if __name__ == '__main__':
#     try:
#         call()
#     except rospy.ROSInterruptException:
#         print("Program interrupted before complrtion")

# -----------------------------------------------------------------------------------------------------------

original_force_x = []
original_force_y = []
original_force_z = []
filtered_force_x_list = []
filtered_force_y_list = []
filtered_force_z_list = []

class KalmanFilter:
    def __init__(self, process_variance, measurement_variance, estimation_error, initial_value=0):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimation_error = estimation_error
        self.estimated_value = initial_value
        self.kalman_gain = 0

    def update(self, measurement):
        self.estimation_error = self.estimation_error + self.process_variance
        self.kalman_gain = self.estimation_error / (self.estimation_error + self.measurement_variance)
        self.estimated_value = self.estimated_value + self.kalman_gain * (measurement - self.estimated_value)
        self.estimation_error = (1 - self.kalman_gain) * self.estimation_error

        return self.estimated_value

ftsensor = None
geomagic = None
kf_x = KalmanFilter(process_variance=2, measurement_variance=6, estimation_error=1)
kf_y = KalmanFilter(process_variance=2, measurement_variance=6, estimation_error=1)
kf_z = KalmanFilter(process_variance=2, measurement_variance=6, estimation_error=1)

# def call():
#     global ftsensor, geomagic
#     rospy.init_node('force_feedback', anonymous=True)
#     ftsensor = rospy.Subscriber('/wrench_topic', Wrench, sensor_callback, queue_size=1)
#     geomagic = rospy.Publisher('/phantom/force_feedback', OmniFeedback, queue_size=1)
#     rate = rospy.Rate(5000)
#     rospy.spin()

def call(subscriber_rate=100000000, publisher_rate=100000000):
    global ftsensor, geomagic
    rospy.init_node('force_feedback', anonymous=True)
    
    # Set up the subscriber with the specified rate
    ftsensor = rospy.Subscriber('/wrench_topic', Wrench, sensor_callback, queue_size=1)
    
    # Set up the publisher with the specified rate
    geomagic = rospy.Publisher('/phantom/force_feedback', OmniFeedback, queue_size=1)
    
    # Set the rates for subscriber and publisher
    subscriber_rate = rospy.Rate(subscriber_rate)
    publisher_rate = rospy.Rate(publisher_rate)
    
    while not rospy.is_shutdown():
        rospy.spin()
        publisher_rate.sleep()  # Sleep to maintain the publisher rate

# def subscriber_callback(data):
#     rospy.loginfo("I heard %s", data.data)

def sensor_callback(msg):
    # geomagic = rospy.Publisher('/phantom/force_feedback', OmniFeedback, queue_size=10)
    # rate = rospy.Rate(1)

    global filtered_force_msg
    global original_force
    global calib_force_x
    original_force = msg.force

    if original_force.x < 5.9:
        calib_force_x = 0
    else :
        calib_force_x = original_force.x - 5.9
    # print("Original force: \n", original_force)
    print("Original force: \n", calib_force_x)

    if original_force.y < -2.5:
        calib_force_y = 0
    else :
        calib_force_y = original_force.y - (-2.5)
    # print("Original force: \n", original_force)
    print("Original force: \n", calib_force_y)

    if original_force.z < -2.05:
        calib_force_z = 0
    else :
        calib_force_z = original_force.z - (-2.05)
    # print("Original force: \n", original_force)
    print("Original force: \n", calib_force_z)

    # filtered_force_x = kf_x.update(original_force.x)
    # filtered_force_y = kf_y.update(original_force.y)
    # filtered_force_z = kf_z.update(original_force.z)

    # filtered_force_msg = Wrench()
    # filtered_force_msg.force.x = filtered_force_x
    # filtered_force_msg.force.y = filtered_force_y
    # filtered_force_msg.force.z = filtered_force_z

    filtered_force_x = kf_x.update(calib_force_x)
    filtered_force_y = kf_y.update(calib_force_y)
    filtered_force_z = kf_z.update(calib_force_z)

    filtered_force_msg = Wrench()
    filtered_force_msg.force.x = filtered_force_x
    filtered_force_msg.force.y = filtered_force_z
    filtered_force_msg.force.z = filtered_force_y

    force_feedback_msg = OmniFeedback()
    force_feedback_msg.force = filtered_force_msg.force
    geomagic.publish(force_feedback_msg)

    print("Filtered force: \n", filtered_force_msg.force)
    print("\n")

    # original_force_x.append(original_force.x)
    # original_force_y.append(original_force.y)
    # original_force_z.append(original_force.z)
    # filtered_force_x_list.append(filtered_force_x)
    # filtered_force_y_list.append(filtered_force_y)
    # filtered_force_z_list.append(filtered_force_z)

    original_force_x.append(calib_force_x)
    original_force_y.append(calib_force_y)
    original_force_z.append(calib_force_z)
    filtered_force_x_list.append(filtered_force_x)
    filtered_force_y_list.append(filtered_force_y)
    filtered_force_z_list.append(filtered_force_z)

def plot_data():
    # time_axis = range(len(original_force_x))
    # plt.figure()
    # plt.subplot(1, 3)
    # plt.plot(time_axis, original_force_x, label='Original Data in X-axis'.format(1))
    # plt.plot(time_axis, filtered_force_x_list, label='Filtered Data in X-axis'.format(1))
    # plt.xlabel('Time (s)')
    # plt.ylabel('Force (N)')
    # plt.legend()
    # plt.title('Force in X-axis'.format(1))

    # plt.figure()
    # plt.plot(time_axis, original_force_y, label='Original Data in Y-axis'.format(2))
    # plt.plot(time_axis, filtered_force_y_list, label='Filtered Data in Y-axis'.format(2))
    # plt.xlabel('Time (s)')
    # plt.ylabel('Force (N)')
    # plt.legend()
    # plt.title('Force in Y-axis'.format(2))

    # plt.figure()
    # plt.plot(time_axis, original_force_z, label='Original Data in Z-axis'.format(3))
    # plt.plot(time_axis, filtered_force_z_list, label='Filtered Data in Z-axis'.format(3))
    # plt.xlabel('Time (s)')
    # plt.ylabel('Force (N)')
    # plt.legend()
    # plt.title('Force in Z-axis'.format(3))

    # plt.tight_layout()
    # plt.show()

    time_axis = range(len(original_force_x))
    
    # Create a single figure with three subplots
    fig, axs = plt.subplots(1, 3, figsize=(15, 5))

    # Plot original and filtered data for the X axis
    axs[0].plot(time_axis, original_force_x, label='Original X')
    axs[0].plot(time_axis, filtered_force_x_list, label='Filtered X')
    axs[0].set_xlabel('Time (s)')
    axs[0].set_ylabel('Force (N)')
    axs[0].legend()
    axs[0].set_title('Force X')

    # Plot original and filtered data for the Y axis
    axs[1].plot(time_axis, original_force_y, label='Original Y')
    axs[1].plot(time_axis, filtered_force_y_list, label='Filtered Y')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('Force (N)')
    axs[1].legend()
    axs[1].set_title('Force Y')

    # Plot original and filtered data for the Z axis
    axs[2].plot(time_axis, original_force_z, label='Original Z')
    axs[2].plot(time_axis, filtered_force_z_list, label='Filtered Z')
    axs[2].set_xlabel('Time (s)')
    axs[2].set_ylabel('Force (N)')
    axs[2].legend()
    axs[2].set_title('Force Z')

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    try:
        call()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion")
        pass
    plot_data()