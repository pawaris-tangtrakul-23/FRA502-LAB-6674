#!/usr/bin/python3

from example_description.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped, Twist
from robot_interfaces.srv import Scheduler, RandomEndeffector, ControllerData
from std_msgs.msg import String
from sensor_msgs.msg import JointState

# Inverse Kinematics
import roboticstoolbox as rtb
from math import pi
from spatialmath import SE3


class RobotSchedulerNode(Node):
    def __init__(self):
        super().__init__("robot_scheduler_node")

        # Set Callback Frequency
        self.declare_parameter("frequency", 100.0)
        self.frequency = (
            self.get_parameter("frequency").get_parameter_value().double_value
        )
        self.create_timer(1 / self.frequency, self.timer_callback)

        # Create Robot state Server and Publisher
        self.random_client = self.create_client(RandomEndeffector, "random_server")
        self.controller_client = self.create_client(ControllerData, "controller_server")
        self.robot_state_server = self.create_service(
            Scheduler, "robot_state_server", self.robot_state_server_callback
        )
        self.robot_state_pub = self.create_publisher(String, "current_state", 10)
        self.current_state = "IDLE"

        # Robot parameters

        self.r_max = 0.28 + 0.25
        self.r_min = 0.03
        self.l = 0.2

        self.get_logger().info("robot_scheduler_node has been started.")

    def req_random(self):
        self.get_logger().info("Request random node")
        state_request = RandomEndeffector.Request()
        state_request.mode.data = "AUTO"
        future = self.random_client.call_async(state_request)
        future.add_done_callback(self.callback_req_random)

    def callback_req_random(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f"Received Response - x: {response.position.x}, y: {response.position.y}, z: {response.position.z}"
            )
            Pe = [
                float(response.position.x),
                float(response.position.y),
                float(response.position.z),
            ]

            if Pe is not None:
                self.get_logger().info(f"End Effector Goal: {Pe}")
                self.req_controller("AUTO", Pe)
            else:
                self.get_logger().error(
                    "Could not compute joint angles for the given position."
                )

        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")

    def req_controller(self, mode, pos):
        try:
            self.get_logger().info("Controller node")
            controller_request = ControllerData.Request()
            controller_request.mode.data = str(mode)
            controller_request.position.x = float(pos[0])
            controller_request.position.y = float(pos[1])
            controller_request.position.z = float(pos[2])

            self.controller_client.call_async(controller_request)
        except Exception as e:
            self.get_logger().error(f"Failed : {e}")
            return None

    def req_ik(self,request):
        try:

            self.get_logger().info(
                f"Received Response - x: {request.position.x}, y: {request.position.y}, z: {request.position.z}"
            )
            q = self.inverse_kinematic(
                float(request.position.x),
                float(request.position.y),
                float(request.position.z),
            )

            if q is not None:
                self.get_logger().info(f"Computed joint angles: {q}")
                Pe = [
                    float(request.position.x),
                    float(request.position.y),
                    float(request.position.z),
                ]

                self.req_controller("IK", Pe)
                return True
            else:
                self.get_logger().error(
                    "Could not compute joint angles for the given position."
                )
                self.current_state = "IDLE"
                return False

        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")
            self.current_state = "IDLE"
            return False

    def robot_state_server_callback(self, request: Scheduler, response: Scheduler):
        self.get_logger().info(
            f"Request To Change State from {str(self.current_state)} to : {str(request.state.data)}"
        )
        self.current_state = str(request.state.data)

        if str(request.state.data) == "AUTO":
            self.req_random()
            response.inprogress = True
            response.mode_readback.data = "AUTO"

        elif str(request.state.data) == "IK":
            response.inprogress = self.req_ik(request)
            response.mode_readback.data = "IK"
            self.get_logger().info(f"response {response.mode_readback.data}")

        elif str(request.state.data) == "TELEOP":
            if str(request.button.data) == "F":
                self.req_controller("TELEOP_F", [0.0, 0.0, 0.0])
            elif str(request.button.data) == "G":
                self.req_controller("TELEOP_G", [0.0, 0.0, 0.0])

            response.inprogress = True

        return response

    def inverse_kinematic(self, x, y, z):
        distance_squared = x**2 + y**2 + (z - 0.2) ** 2
        if self.r_min**2 <= distance_squared <= self.r_max**2:

            robot = rtb.DHRobot(
                [
                    rtb.RevoluteMDH(alpha=0.0, a=0.0, d=0.2, offset=0.0),
                    rtb.RevoluteMDH(alpha=pi / 2, a=0.0, d=0.02, offset=0.0),
                    rtb.RevoluteMDH(alpha=0, a=0.25, d=0.0, offset=0.0),
                ],
                tool=SE3.Tx(0.28),
                name="RRR_Robot",
            )

            T_Position = SE3(x, y, z)
            ik_solution = robot.ikine_LM(
                T_Position, mask=[1, 1, 1, 0, 0, 0], joint_limits=False, q0=[0, 0, 0]
            )

            if ik_solution.success:
                q_sol_ik_LM = ik_solution.q  # Extract joint angles
                return q_sol_ik_LM
            else:
                self.get_logger().error("Inverse kinematics failed to find a solution.")
                return None
        else:
            self.get_logger().error("Target position is out of reach.")
            return None

    def timer_callback(self):
        msg = String()
        msg.data = self.current_state
        self.robot_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotSchedulerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
