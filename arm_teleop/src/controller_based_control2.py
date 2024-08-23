#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import pygame
import time

#Axis 2 is uselessly noisy, so we're going to ignore it
#Axis 0,1 are left stick
#Axis 3,4 are right stick

#Button 4 is L1
#Button 5 is R1
#Button 6 is L2
#Button 7 is R2


class XboxController:
    def __init__(self, joystick_id=0, correction_threshold=0.1, correction_rate=0.1):
        # Initialize pygame and joystick
        pygame.init()
        pygame.joystick.init()

        # Check if the joystick is connected
        if pygame.joystick.get_count() < 1:
            raise ValueError("No joystick connected!")

        # Initialize the joystick
        self.joystick = pygame.joystick.Joystick(joystick_id)
        self.joystick.init()

        # Zero error dictionary for each axis
        self.zero_error = {}
        self.correction_threshold = correction_threshold
        self.correction_rate = correction_rate

    def get_raw_axis_values(self):
        """Get the current raw axis values from the controller."""
        pygame.event.pump()  # Process event queue
        axis_values = {}
        for i in range(self.joystick.get_numaxes()):
            axis_value = self.joystick.get_axis(i)
            axis_values[f'Axis {i}'] = axis_value
        return axis_values

    def update_zero_error(self, axis_values):
        """Update the zero error corrections dynamically based on the current axis values."""
        for axis, value in axis_values.items():
            # Initialize zero error if not already set
            if axis not in self.zero_error:
                self.zero_error[axis] = value

            # Adjust zero error slowly if the value is close to the current zero error
            if abs(value - self.zero_error[axis]) < self.correction_threshold:
                self.zero_error[axis] = (
                    self.zero_error[axis] * (1 - self.correction_rate) +
                    value * self.correction_rate
                )

    def get_corrected_axis_values(self):
        """Get the corrected axis values after removing zero error."""
        raw_values = self.get_raw_axis_values()
        self.update_zero_error(raw_values)

        corrected_values = {}
        for axis, value in raw_values.items():
            corrected_value = value - self.zero_error[axis]
            corrected_value = max(-1.0, min(1.0, corrected_value))  # Clamp value between -1 and 1
            corrected_values[axis] = round(corrected_value, 5)  # Round to 5 decimal places

        return corrected_values
    
    def get_button_state(self):
        """Get the state of the R1 (RB), R2 (RT), L1 (LB), and L2 (LT) buttons."""
        pygame.event.pump()  # Process event queue

        # R1 (RB) is usually button 5 on Xbox controllers
        r1_pressed = self.joystick.get_button(5)

        # R2 (RT) is usually button 7 on Xbox controllers
        r2_pressed = self.joystick.get_button(7)

        # L1 (LB) is usually button 4 on Xbox controllers
        l1_pressed = self.joystick.get_button(4)

        # L2 (LT) is usually button 6 on Xbox controllers
        l2_pressed = self.joystick.get_button(6)

        return {
            'R1': r1_pressed,
            'R2': r2_pressed,
            'L1': l1_pressed,
            'L2': l2_pressed
        }


    def close(self):
        """Clean up and close the joystick and pygame."""
        self.joystick.quit()
        pygame.quit()

def main():
    rospy.init_node('send_joint_trajectory')
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

    controller = XboxController()

    # Initialize joint positions
    joint_positions = {
        '0': 0.0,
        '1': 0.0,
        '2': 0.0,
        '3': 0.0,
        '4': 0.0
    }

    try:
        while not rospy.is_shutdown():
            corrected_axes = controller.get_corrected_axis_values()
            button_state = controller.get_button_state()

            # Add axis values to the current joint positions
            joint_positions['0'] += corrected_axes.get('Axis 0', 0.0) * 0.1
            joint_positions['1'] += corrected_axes.get('Axis 1', 0.0) * 0.1
            joint_positions['2'] += corrected_axes.get('Axis 3', 0.0) * 0.1
            joint_positions['3'] += corrected_axes.get('Axis 4', 0.0) * 0.1

            # Handle R1 and R2 button input for joint 4
            if button_state['R1']:
                joint_positions['4'] += 0.1
            elif button_state['R2']:
                joint_positions['4'] -= 0.1

            # # Clamp the joint values to within the allowed range (if necessary)
            # # For example, to limit joint movement between -1.0 and 1.0:
            # for joint in joint_positions:
            #     joint_positions[joint] = max(-1.0, min(1.0, joint_positions[joint]))

            # Create a JointTrajectory message
            trajectory_msg = JointTrajectory()
            trajectory_msg.joint_names = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4']

            # Create a single trajectory point
            point = JointTrajectoryPoint()
            point.positions = [joint_positions[str(i)] for i in range(5)]
            point.time_from_start = rospy.Duration(1.0)

            # Add the point to the trajectory
            trajectory_msg.points.append(point)

            # Publish the trajectory
            pub.publish(trajectory_msg)
            rospy.loginfo(f"Trajectory command sent with positions {joint_positions}")

            time.sleep(0.1)  # Delay to control the update rate

    except KeyboardInterrupt:
        print("Exiting...")

    finally:
        controller.close()

if __name__ == "__main__":
    main()
