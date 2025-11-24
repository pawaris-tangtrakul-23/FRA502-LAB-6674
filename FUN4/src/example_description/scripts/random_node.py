#!/usr/bin/python3

from example_description.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist, Point, TransformStamped, PoseStamped
from turtlesim.msg import Pose
from std_msgs.msg import String
from robot_interfaces.srv import Scheduler,RandomEndeffector
import random

import roboticstoolbox as rtb
from math import pi
from spatialmath import SE3
import numpy as np

class RandomNode(Node):
    def __init__(self):
        super().__init__('random_node')
        # Set Callback Frequency
        self.declare_parameter("frequency", 100.0)
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        self.create_timer(1 / self.frequency, self.timer_callback)

        self.target_pub = self.create_publisher(PoseStamped, 'target', 10)

        # Set Subscribe Current State
        self.create_subscription(String, "/current_state", self.current_state_callback, 10)
        self.scheduler_client = self.create_client(Scheduler, "robot_state_server")

        # Create Random Server for calculate inverse kinematics
        self.random_server = self.create_service(RandomEndeffector, "random_server", self.random_server_callback)
        self.r_max = 0.28+0.25
        self.r_min = 0.03
        self.l = 0.2

        self.current_state = "IDLE"
        self.get_logger().info("random_node has been started.")

    def req_scheduler(self, state):
        self.get_logger().info("Request robot_scheduler node")
        state_request = Scheduler.Request()
        state_request.state.data = str(state)
        future = self.scheduler_client.call_async(state_request)

    def random_server_callback(self, request:RandomEndeffector, response:RandomEndeffector):

        self.get_logger().info(f"Request {str(request.mode.data)} mode")
        if request.mode.data == "AUTO":

            while True:
                x = random.uniform(-self.r_max, self.r_max)
                y = random.uniform(-self.r_max, self.r_max)
                z = random.uniform(-self.r_max, self.r_max)

                size_squared = x**2 + y**2 + (z - self.l)**2

                if self.r_min**2 < size_squared < self.r_max**2:
                    try:
                        goal = self.inverse_kinematic(x,y,z)
                        self.goal = [x, y, z]

                        msg = PoseStamped()
                        msg.pose.position.x = x
                        msg.pose.position.y = y
                        msg.pose.position.z = z

                        response.position.x = x
                        response.position.y = y
                        response.position.z = z
                        response.inprogress = True

                        # response.message = f"Target set at: {x, y, z}"
                        self.target_pub.publish(msg)
                        break
                    except Exception as e:
                        self.get_logger().error(f"Computation Error: {e}")
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
                self.req_scheduler("IDLE")
                return None
        else:
            self.get_logger().error("Target position is out of reach.")
            return None

    def current_state_callback(self, msg:String):
        self.current_state = msg.data      

    def timer_callback(self):
        # print(self.target)
        pass

def main(args=None):
    rclpy.init(args=args)
    node = RandomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
