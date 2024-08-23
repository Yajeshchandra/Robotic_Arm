import pygame
import time
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
import sys

# Xbox Controller mappings
LEFT_STICK_X = "Axis 0"
LEFT_STICK_Y = "Axis 1"
RIGHT_STICK_X = "Axis 3"
RIGHT_STICK_Y = "Axis 4"

# Button mappings
L1_BUTTON = 4

# Scaling factor for translation (adjust as necessary)
SCALE = 0.01

class XboxController:
    def __init__(self, joystick_id=0, correction_threshold=0.1, correction_rate=0.1):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() < 1:
            raise ValueError("No joystick connected!")
        self.joystick = pygame.joystick.Joystick(joystick_id)
        self.joystick.init()
        self.zero_error = {}
        self.correction_threshold = correction_threshold
        self.correction_rate = correction_rate

    def get_raw_axis_values(self):
        pygame.event.pump()
        axis_values = {}
        for i in range(self.joystick.get_numaxes()):
            axis_value = self.joystick.get_axis(i)
            axis_values[f'Axis {i}'] = axis_value
        return axis_values

    def update_zero_error(self, axis_values):
        for axis, value in axis_values.items():
            if axis not in self.zero_error:
                self.zero_error[axis] = value
            if abs(value - self.zero_error[axis]) < self.correction_threshold:
                self.zero_error[axis] = (
                    self.zero_error[axis] * (1 - self.correction_rate) +
                    value * self.correction_rate
                )

    def get_corrected_axis_values(self):
        raw_values = self.get_raw_axis_values()
        self.update_zero_error(raw_values)
        corrected_values = {}
        for axis, value in raw_values.items():
            corrected_value = value - self.zero_error[axis]
            corrected_value = max(-1.0, min(1.0, corrected_value))
            corrected_values[axis] = round(corrected_value, 5)
        return corrected_values
    
    def get_button_state(self):
        pygame.event.pump()
        return self.joystick.get_button(L1_BUTTON)

    def close(self):
        self.joystick.quit()
        pygame.quit()

class RobotController:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('xbox_arm_controller', anonymous=True)
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        rospy.loginfo("RobotController initialized.")

    def get_current_pose(self):
        pose = self.arm.get_current_pose().pose
        rospy.loginfo(f"Current end-effector pose: {pose}")
        return pose

    def move_to_pose(self, pose):
        rospy.loginfo(f"Moving to pose: {pose}")
        self.arm.set_pose_target(pose)
        success = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        rospy.loginfo(f"Move successful: {success}")

    def publish_marker(self, position):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = position
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        rospy.loginfo(f"Publishing marker at position: {position}")
        self.marker_pub.publish(marker)

if __name__ == "__main__":
    try:
        controller = XboxController()
        robot_controller = RobotController()

        while not rospy.is_shutdown():
            corrected_axes = controller.get_corrected_axis_values()
            rospy.loginfo(f"Corrected axis values: {corrected_axes}")
            l1_pressed = controller.get_button_state()
            rospy.loginfo(f"L1 button pressed: {l1_pressed}")

            current_pose = robot_controller.get_current_pose()

            new_position = Point()
            new_position.x = current_pose.position.x + corrected_axes[LEFT_STICK_X] * SCALE
            new_position.y = current_pose.position.y + corrected_axes[LEFT_STICK_Y] * SCALE
            new_position.z = current_pose.position.z + corrected_axes[RIGHT_STICK_Y] * SCALE

            rospy.loginfo(f"New target position: {new_position}")

            robot_controller.publish_marker(new_position)

            if l1_pressed:
                rospy.loginfo("L1 button pressed, executing move.")
                current_pose.position = new_position
                robot_controller.move_to_pose(current_pose)

            time.sleep(1)

    except rospy.ROSInterruptException:
        pass

    finally:
        controller.close()
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("Shutdown complete.")
