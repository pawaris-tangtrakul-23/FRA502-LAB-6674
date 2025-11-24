#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty
from controller_manager_msgs.srv import LoadController, ConfigureController
from robot_interfaces.srv import Scheduler
from std_msgs.msg import String


class UINode(Node):
    def __init__(self):
        super().__init__("ui_node")

        # Set Callback Frequency to 10 Hz
        self.declare_parameter("frequency", 10.0)
        self.frequency = (
            self.get_parameter("frequency").get_parameter_value().double_value
        )
        self.create_timer(1 / self.frequency, self.timer_callback)

        # Subscribe to Current State
        self.create_subscription(
            String, "/current_state", self.current_state_callback, 10
        )
        self.scheduler_client = self.create_client(Scheduler, "robot_state_server")

        # Terminal settings for keyboard input
        self.settings = termios.tcgetattr(sys.stdin)
        self.current_state = "IDLE"
        self.get_logger().info("ui_node has been started.")

    def req_scheduler(self, state, button="", pos=[0.0, 0.0, 0.0]):
        self.get_logger().info(
            f"Request robot_scheduler node with state: {state}"
        )
        state_request = Scheduler.Request()
        state_request.state.data = str(state)
        state_request.button.data = str(button)
        state_request.position.x = float(pos[0])
        state_request.position.y = float(pos[1])
        state_request.position.z = float(pos[2])

        # Call the service asynchronously
        future = self.scheduler_client.call_async(state_request)
        future.add_done_callback(self.callback_req_scheduler)

    def current_state_callback(self, msg: String):
        self.current_state = msg.data

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def callback_req_scheduler(self, future):
        try:
            response = future.result()
            # self.get_logger().info(f"Scheduler Response: {response}")
            # self.get_logger().info(f"Mode Readback: {response.mode_readback.data}")
            if not response.inprogress and response.mode_readback.data == "IK":
                self.get_logger().error(
                    "Could not compute joint angles for the given position."
                )
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")

    def timer_callback(self):
        try:
            key = self.getKey()

            if key is None:
                return  # No key was pressed; do nothing

            if key == "a":
                self.get_logger().info("Request Mode AUTO")
                self.req_scheduler("AUTO")
            elif key == "s":
                self.get_logger().info("Request Mode IK")

                # List to store the three decimal values
                p_e = []

                # Collect three decimal numbers
                for i in range(3):
                    value = ""
                    self.get_logger().info(f"Enter decimal number {i+1}: ")

                    # Loop to collect the value for each decimal number
                    while True:
                        key_input = self.getKey()
                        if key_input is None:
                            continue  # Wait for input

                        if key_input == "\x03":  # Ctrl+C to exit
                            self.req_scheduler("IDLE")
                            rclpy.shutdown()
                            return

                        if key_input in ("\r", "\n"):  # Enter key pressed
                            try:
                                # Convert to float and add to list
                                p_e.append(float(value))
                                self.get_logger().info(
                                    f"Value {i+1} confirmed: {value}"
                                )
                                break
                            except ValueError:
                                self.get_logger().info(
                                    "Invalid input. Please enter a valid decimal number."
                                )
                                value = ""  # Reset value for re-entering

                        elif key_input == "\x7f":  # Backspace key pressed
                            value = value[:-1]  # Remove last character
                            print(f"\r{' '*30}\r{value}", end="", flush=True)

                        else:
                            value += key_input  # Append key to value
                            print(f"\r{value}", end="", flush=True)

                self.get_logger().info(f"Collected decimal values: {p_e}")

                # Call req_scheduler without additional add_done_callback
                self.req_scheduler("IK", pos=p_e)

            elif key == "f":
                self.get_logger().info("Request Mode TELEOP: Moving around")
                self.req_scheduler("TELEOP", button="F")
            elif key == "g":
                self.get_logger().info(
                    "Request Mode TELEOP: For Holonomic mode (strafing), hold down the shift key."
                )
                self.req_scheduler("TELEOP", button="G")
            else:
                self.get_logger().info("Unknown key pressed. No action taken.")

            if key == "\x03":  # Ctrl+C to exit
                self.req_scheduler("IDLE")
                rclpy.shutdown()
                return

        except Exception as e:
            self.get_logger().error(f"Failed in timer_callback: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = UINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
