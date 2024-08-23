import ikpy.chain
import rospy
from ikpy.utils import plot
from ikpy import urdf
import numpy as np
import matplotlib.pyplot as plt

# Load your robot's URDF file
robot_chain = ikpy.chain.Chain.from_urdf_file("arm_urdf/urdf/arm_urdf.urdf")

# Display the initial pose of the robot
initial_pose = robot_chain.forward_kinematics([0] * robot_chain.number_of_joints)
print(f"Initial pose:\n{initial_pose}")

# Plot the initial pose if you want to visualize it
# fig, ax = plot.init_3d_figure()
# robot_chain.plot([0] * robot_chain.number_of_joints, ax, target=None)
# plt.show()
new_pose = initial_pose.copy()
new_pose.position.x+=0.1
new_pose.position.y+=0.1
new_pose.position.z+=0.1
# Define your target position (use your new_pose)
target_position = [new_pose.position.x, new_pose.position.y, new_pose.position.z]

# Compute the joint angles using inverse kinematics
joint_angles = robot_chain.inverse_kinematics(target_position)

# Log the computed joint angles
rospy.loginfo(f"Computed joint angles: {joint_angles}")
