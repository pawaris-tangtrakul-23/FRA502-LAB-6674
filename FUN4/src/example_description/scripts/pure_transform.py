#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from robot_interfaces.srv import RandomEndeffector
import numpy as np

import roboticstoolbox as rtb
from math import pi
from spatialmath import SE3


class PureTransformNode(Node):
    def __init__(self):
        super().__init__("pure_transform_node")

        # Set Callback Frequency
        self.declare_parameter("frequency", 0.1)
        self.frequency = (
            self.get_parameter("frequency").get_parameter_value().double_value
        )
        self.create_timer(1 / self.frequency, self.timer_callback)

        # Create Service Client for RandomEndeffector
        self.random_client = self.create_client(RandomEndeffector, "random_server")
        while not self.random_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for 'random_server' service...")

        # Create JointState Publisher
        self.joint_state_publisher = self.create_publisher(
            JointState, "joint_states", 10
        )
        self.joint_state = JointState()
        self.joint_state.header.frame_id = ""
        self.joint_state.name = ["joint_1", "joint_2", "joint_3"]
        self.joint_state.position = [0.0, 0.0, 0.0]

        self.r_max = 0.28 + 0.25
        self.r_min = 0.03
        self.l = 0.2

        self.get_logger().info("pure_transform_node has been started.")

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
            q = self.inverse_kinematic(
                float(response.position.x),
                float(response.position.y),
                float(response.position.z)
            )

            if q is not None:
                self.get_logger().info(f"Computed joint angles: {q}")
                self.publish_joint_state(q)
            else:
                self.get_logger().error(
                    "Could not compute joint angles for the given position."
                )

        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")

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

    def publish_joint_state(self, positions):
        if len(positions) != len(self.joint_state.name):
            self.get_logger().error(
                f"Number of positions ({len(positions)}) does not match number of joints ({len(self.joint_state.name)})."
            )
            return

        self.joint_state.header.stamp = self.get_clock().now().to_msg()

        # Update joint positions
        self.joint_state.position = positions.tolist()  # Ensure it's a list

        self.joint_state_publisher.publish(self.joint_state)
        self.get_logger().info("Published JointState message.")

    def timer_callback(self):
        self.req_random()


def main(args=None):
    rclpy.init(args=args)
    node = PureTransformNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
