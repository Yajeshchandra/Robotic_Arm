#!/usr/bin/python

import rospy
import moveit_commander
from geometry_msgs.msg import Pose
import sys

class TestRobotArm:
    def __init__(self):
        # Initialize the MoveIt commander
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("test_robot_arm", anonymous=True)

        # Initialize the robot and the move group
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm")  # Replace "arm" with your robot's group name

        # Get and display the current pose of the end effector
        self.current_pose = self.group.get_current_pose().pose
        self.display_current_pose()

    def display_current_pose(self):
        print("Current End Effector Position:")
        print(f"X: {self.current_pose.position.x}")
        print(f"Y: {self.current_pose.position.y}")
        print(f"Z: {self.current_pose.position.z}")
        print(f"Orientation:")
        print(f"X: {self.current_pose.orientation.x}")
        print(f"Y: {self.current_pose.orientation.y}")
        print(f"Z: {self.current_pose.orientation.z}")
        print(f"W: {self.current_pose.orientation.w}")

    def get_new_pose_from_input(self):
        new_pose = Pose()
        try:
            # Get new position from the user
            new_pose.position.x = float(input("Enter new X position: "))
            new_pose.position.y = float(input("Enter new Y position: "))
            new_pose.position.z = float(input("Enter new Z position: "))

            # Get new orientation from the user (if you want to rotate the end effector)
            new_pose.orientation.x = float(input("Enter new orientation X: ")) 
            new_pose.orientation.y = float(input("Enter new orientation Y: "))
            new_pose.orientation.z = float(input("Enter new orientation Z: "))
            new_pose.orientation.w = float(input("Enter new orientation W: "))

            return new_pose
        except ValueError:
            print("Invalid input. Please enter numerical values.")
            return None

    def move_to_new_pose(self, new_pose):
        if new_pose:
            # Set the target pose for the end effector
            self.group.set_pose_target(new_pose)

            # Plan and execute the motion
            plan = self.group.plan()
            if plan:
                print("Planning successful. Executing...")
                self.group.go(wait=True)
                print("Movement executed.")
            else:
                print("Planning failed.")
        else:
            print("No valid pose provided. Aborting.")

if __name__ == "__main__":
    arm_tester = TestRobotArm()

    while not rospy.is_shutdown():
        # Display the current position
        arm_tester.display_current_pose()

        # Get a new position from the user
        new_pose = arm_tester.get_new_pose_from_input()

        # Move to the new position
        arm_tester.move_to_new_pose(new_pose)

        # Ask if the user wants to continue or exit
        continue_input = input("Do you want to enter another position? (y/n): ")
        if continue_input.lower() != 'y':
            print("Exiting.")
            break

    moveit_commander.roscpp_shutdown()
