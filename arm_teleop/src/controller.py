#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pynput import keyboard

# Track the selected joint and its position
selected_joint = '0'
joint_positions = {
  '0': 0.0,
  '1': 0.0,
  '2': 0.0,
  '3': 0.0,
  '4': 0.0
}

def on_press(key):
  global selected_joint

  try:
    # Check if the key corresponds to a joint
    if key.char in joint_positions:
      selected_joint = key.char
      rospy.loginfo(f"Selected joint {selected_joint}")

    # Increment or decrement the joint position with 'w' and 's'
    elif key.char == 'w':
      joint_positions[selected_joint] += 0.1
      rospy.loginfo(f"Moving joint {selected_joint} to position {joint_positions[selected_joint]}")

      #now publish the joint positions

      # Create a JointTrajectory message
      trajectory_msg = JointTrajectory()
      trajectory_msg.joint_names = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4']  # Specify your joint names here

      # Create a single trajectory point
      point = JointTrajectoryPoint()
      point.positions = joint_positions.values()  # Desired joint positions
      point.time_from_start = rospy.Duration(1.0)  # Time to reach the point (1 second)

      # Add the point to the trajectory
      trajectory_msg.points.append(point)

      # Publish the trajectory
      pub.publish(trajectory_msg)

      rospy.loginfo("Trajectory command sent!")

    elif key.char == 's':
      joint_positions[selected_joint] -= 0.1
      rospy.loginfo(f"Moving joint {selected_joint} to position {joint_positions[selected_joint]}")
      #now publish the joint positions

      # Create a JointTrajectory message
      trajectory_msg = JointTrajectory()
      trajectory_msg.joint_names = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4']  # Specify your joint names here

      # Create a single trajectory point
      point = JointTrajectoryPoint()
      point.positions = joint_positions.values()  # Desired joint positions
      point.time_from_start = rospy.Duration(1.0)  # Time to reach the point (1 second)

      # Add the point to the trajectory
      trajectory_msg.points.append(point)

      # Publish the trajectory
      pub.publish(trajectory_msg)

      rospy.loginfo("Trajectory command sent!")


  except AttributeError:
    pass

def on_release(key):
  # Stop listener on 'esc' key
  if key == keyboard.Key.esc:
    rospy.loginfo("Exiting teleoperation")
    return False

rospy.init_node('send_joint_trajectory')

# Create a publisher for the trajectory
pub = rospy.Publisher('/robot_arm_controller/command', JointTrajectory, queue_size=10)

# Wait until a subscriber connects to the publisher
i = 0
while not rospy.is_shutdown() and pub.get_num_connections() == 0:
    if i == 4:
        print("Waiting for subscriber to connect to {}".format(pub.name))
    rospy.sleep(0.5)
    i += 1
    i = i % 5
if rospy.is_shutdown():
    raise Exception("Got shutdown request before subscribers connected")

# Start listening to keyboard events
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
  listener.join()
