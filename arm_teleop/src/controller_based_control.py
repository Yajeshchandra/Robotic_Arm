#!/usr/bin/python

import rospy
import moveit_commander
from geometry_msgs.msg import Pose, Twist
from inputs import get_gamepad
import math
import threading
import tf
import sys

class XboxController(object):
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):
        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.RightBumper = 0

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def read(self):
        x = self.LeftJoystickX
        y = self.LeftJoystickY
        rx = self.RightJoystickY
        ry = self.RightJoystickX
        r1 = self.RightBumper
        return [x, y, rx, ry, r1]

    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                if event.code == 'ABS_Y':
                    self.LeftJoystickY = event.state / XboxController.MAX_JOY_VAL  # -1 to 1
                elif event.code == 'ABS_X':
                    self.LeftJoystickX = event.state / XboxController.MAX_JOY_VAL  # -1 to 1
                elif event.code == 'ABS_RY':
                    self.RightJoystickY = event.state / XboxController.MAX_JOY_VAL  # -1 to 1
                elif event.code == 'ABS_RX':
                    self.RightJoystickX = event.state / XboxController.MAX_JOY_VAL  # -1 to 1
                elif event.code == 'BTN_TR':
                    self.RightBumper = event.state


class RobotArmController:
    def __init__(self):
        # Initialize MoveIt Commander
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("robot_arm_controller")

        # Initialize robot and move group
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm")  # Replace "arm" with your robot's group name

        # Get initial pose of the end effector
        self.current_pose = self.group.get_current_pose().pose

        # Initialize Xbox controller interface
        self.controller = XboxController()
        self.listener = threading.Thread(target=self._monitor_controller)
        self.listener.start()

    def update_end_effector_pose(self, delta_x, delta_y, delta_z, delta_yaw):
        self.current_pose.position.x += delta_x
        self.current_pose.position.y += delta_y
        self.current_pose.position.z += delta_z

        # Adjust orientation for yaw rotation
        quaternion = self.group.get_current_pose().pose.orientation
        current_yaw = self._quaternion_to_euler(quaternion).z
        new_yaw = current_yaw + delta_yaw
        self.current_pose.orientation = self._euler_to_quaternion(0, 0, new_yaw)

        self.group.set_pose_target(self.current_pose)
        self.group.go(wait=True)

    def _monitor_controller(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            [lx, ly, _, rx, r1] = self.controller.read()
            delta_x = lx * 0.01  # Adjust step size as needed
            delta_y = ly * 0.01
            delta_z = rx * 0.01
            delta_yaw = rx * 0.01
            self.update_end_effector_pose(delta_x, delta_y, delta_z, delta_yaw)
            
            if r1:  # R1 button pressed for gripper control
                self.control_gripper()

            rate.sleep()

    def control_gripper(self):
        # Add code to open/close gripper here
        
        pass

    def _quaternion_to_euler(self, quaternion):
        # Convert quaternion to euler angles
        euler = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return euler

    def _euler_to_quaternion(self, roll, pitch, yaw):
        # Convert euler angles to quaternion
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        return quaternion

if __name__ == "__main__":
    arm_controller = RobotArmController()
    rospy.spin()
