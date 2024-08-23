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


if __name__ == "__main__":
    try:
        # Instantiate the XboxController class
        controller = XboxController()

        while True:
            corrected_axes = controller.get_corrected_axis_values()
            button_state = controller.get_button_state()
            print("Corrected axis values:", corrected_axes)
            print("Button state:", button_state)
            time.sleep(0.1)  # Delay to prevent flooding the terminal

    except KeyboardInterrupt:
        print("Exiting...")

    finally:
        # Clean up
        controller.close()
