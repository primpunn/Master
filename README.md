# Master
This work presents a teleoperation system that allows for remote control of a UR5 robot arm using a Geomagic Touch haptic device. The system translates human hand movements to the robot, providing precise control while offering real-time haptic feedback. This feedback allows the operator to feel the force exerted by the robot, enhancing the accuracy and immersiveness of the teleoperation.

The control is achieved by scaling and transmitting joint positions from the haptic device to the robot arm, while a force-torque sensor on the robot measures applied forces. This data is relayed back to the haptic device at a frequency of 500 Hz to ensure real-time responsiveness. A Kalman filter is used to minimize noise in the force data, ensuring reliable feedback.

Experimental results show minimal delay less than 1 second and strong force feedback, demonstrating the system’s potential, especially in medical applications like remote ultrasound procedures. The research contributes to teleoperation technology, offering a framework for real-time control and feedback that could make healthcare more accessible, particularly in underserved areas.
