#!/usr/bin/python3

import rclpy
from rclpy.node import Node

# Services and Messages
from robot_interfaces.srv import Random
from geometry_msgs.msg import PoseStamped

# Logic imports
import random
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3, UnitQuaternion


class RandomServiceNode(Node):
    def __init__(self):
        super().__init__("random_service_node")

        # Create Service and Publisher
        self.srv = self.create_service(Random, "random", self.random_callback)
        self.target_pub = self.create_publisher(PoseStamped, "/target", 10)

        # --- ROBOT DEFINITION (Same as control.py) ---
        # We define the robot here to use its Forward Kinematics
        self.robot = rtb.DHRobot(
            [
                rtb.RevoluteMDH(alpha=0.0, a=0.0, d=0.2, offset=0.0),
                rtb.RevoluteMDH(alpha=np.pi / 2, a=0.0, d=0.02, offset=0.0),
                rtb.RevoluteMDH(alpha=0.0, a=0.25, d=0.0, offset=0.0),
            ],
            tool=SE3.Tx(0.28),
            name="3R_Robot",
        )

        self.get_logger().info("Random Position Service (FK-Based) is Ready.")

    def random_callback(self, request, response):
        # --- LOGIC TO GENERATE VALID TARGET ---

        valid_pose = False

        while not valid_pose:
            # 1. Generate Random Joint Angles
            # Limits can be adjusted, using -pi to pi for full range
            q_rand = [
                random.uniform(-np.pi, np.pi),  # Joint 1
                random.uniform(-np.pi, np.pi),  # Joint 2
                random.uniform(-np.pi, np.pi),  # Joint 3
            ]

            # 2. Compute Forward Kinematics to get Pose
            T_rand = self.robot.fkine(q_rand)

            x = T_rand.t[0]
            y = T_rand.t[1]
            z = T_rand.t[2]

            # 3. Check Workspace Limits (From Lab Req / control.py)
            # Center of workspace is at shoulder (0, 0, 0.2)
            z_offset = z - 0.2
            dist_sq = x**2 + y**2 + z_offset**2

            # Limits: 0.03m to 0.53m
            if 0.03**2 <= dist_sq <= 0.53**2:
                # Also prevent hitting the floor if necessary (z >= 0)
                if z >= 0.0:
                    valid_pose = True

        # --- PUBLISH TARGET ---
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "link_0"

        # Position
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        # Orientation (Quaternions from Rotation Matrix)
        quat = UnitQuaternion(T_rand.R)
        msg.pose.orientation.w = float(quat.s)
        msg.pose.orientation.x = float(quat.v[0])
        msg.pose.orientation.y = float(quat.v[1])
        msg.pose.orientation.z = float(quat.v[2])

        self.target_pub.publish(msg)

        # --- SERVICE RESPONSE ---
        response.position.x = x
        response.position.y = y
        response.position.z = z
        response.success = True

        self.get_logger().info(f"Generated Target: [{x:.3f}, {y:.3f}, {z:.3f}]")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = RandomServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
