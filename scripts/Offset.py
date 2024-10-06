#!/usr/bin/env python
from ikpy.chain import Chain
# from ikpy.chain import find_chain
import numpy as np

# Load the robot models
geomagic_chain = Chain.from_urdf_file('/home/primpunn/catkin_ws/src/phantom_omni/omni_description/urdf/omni.urdf', base_elements=["base"], active_links_mask=[False, True, True, True, True, True, False])
ur5_chain = Chain.from_urdf_file('/home/primpunn/catkin_ws/src/test/config/gazebo_ur5_robot.urdf')

# verify loaded chain links
print("Geomagic Touch Links: ", [link.name for link in geomagic_chain.links])
print("UR5 Robot Arm Links: ", [link.name for link in ur5_chain.links])

target_positions = [
    [0.051, 0.193, -2.474],
    [2.091, -3.47, 0.073],
    [1.921, -2.681, 0.51]
]

# Function to compute joint angles for each target position
def compute_joint_angles(chain, target_position):
    assert len(target_position) == 3, "Target position must be a 3D vector"
    target_transform = np.eye(4)
    target_transform[:3, 3] = target_position
    print(f"Target Transform for {chain}: \n{target_transform}")
    # if initial_guess is None:
    #     initial_guess = chain.active_joint_positions
    try:
        position_only = target_transform[:3, 3]
        result = chain.inverse_kinematics(position_only)
        print(f"Joint Angles: {result}")
        return result
    except Exception as e:
        print(f"Error in inverse_kinematics: {e}")
        return None

geomagic_joint_angles_list = []
ur5_joint_angles_list = []

for i, pos in enumerate(target_positions):
    geomagic_joint_angles = compute_joint_angles(geomagic_chain, pos)
    ur5_joint_angles = compute_joint_angles(ur5_chain, pos)
    
    if geomagic_joint_angles is None or ur5_joint_angles is None:
        print(f"Skipping target position {target_positions[i]} due to IK error.")
        continue

    geomagic_joint_angles_list.append(geomagic_joint_angles)
    ur5_joint_angles_list.append(ur5_joint_angles)

# Compute offsets for each joint at each target position
offsets_list = []
for i in range(len(geomagic_joint_angles_list)):
    geomagic_angles = geomagic_joint_angles_list[i]
    ur5_angles = ur5_joint_angles_list[i]

    min_length = min(len(ur5_joint_angles), len(geomagic_joint_angles))
    offsets = [ur5_angles[j] - geomagic_angles[j] for j in range(min_length)]
    offsets_list.append(offsets)

print("Offsets for each target position:", offsets_list)
