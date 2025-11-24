#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from robot_interfaces.srv import Mode
from robot_interfaces.srv import Random
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
import roboticstoolbox as rtb
from math import pi, cos, sin
from spatialmath import SE3, UnitQuaternion
import numpy as np
import random


class DummyNode(Node):
    def __init__(self):
        super().__init__("control_node")

        # Publishers
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.fk_pub = self.create_publisher(PoseStamped, "/end_effector", 10)

        # Subscribers
        self.cmd_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )
        self.frame_sub = self.create_subscription(
            String, "/teleop_frame", self.frame_callback, 10
        )

        # Services
        self.mode_srv = self.create_service(Mode, "mode", self.mode_callback)
        self.cli_random = self.create_client(Random, "random")
        self.req_random = Random.Request()

        # Internal State
        self.joint = [0.0, 0.5, 0.5]
        self.joint_names = ["joint_1", "joint_2", "joint_3"]

        # TO Mode Variables
        self.current_vel = np.zeros(6)
        self.control_frame = "world"

        # Trajectory Variables
        self.mode = 0
        self.traj_q = []
        self.traj_index = 0
        self.count = 0

        # Robot Definition
        self.robot = rtb.DHRobot(
            [
                rtb.RevoluteMDH(alpha=0.0, a=0.0, d=0.2, offset=0.0),
                rtb.RevoluteMDH(alpha=-pi / 2, a=0.0, d=-0.12, offset=-pi / 2),
                rtb.RevoluteMDH(alpha=0.0, a=0.25, d=0.1, offset=0.0),
            ],
            tool=SE3.Tx(0.28),
            name="3R_Robot",
        )
        # Timer
        self.dt = 0.01
        self.create_timer(self.dt, self.control_loop)

    def control_loop(self):
        # --- MODE 2: TELEOPERATION ---
        if self.mode == 2:
            # 1. Get Jacobian
            if self.control_frame == "world":
                J = self.robot.jacob0(self.joint)
            else:
                J = self.robot.jacobe(self.joint)

            # 2. Singularity Check
            J_pos = J[:3, :]
            det_J = np.linalg.det(J_pos)

            if abs(det_J) < 0.0001:
                self.get_logger().warn(f"SINGULARITY! Det: {det_J:.5f}")
                q_dot = np.zeros(3)
            else:
                # 3. Calculate Joint Velocity
                v_desired = self.current_vel[:3]
                q_dot = np.linalg.pinv(J_pos) @ v_desired

            # --- NEW: RANGE CHECK ---
            # Predict next position
            next_joint = (self.joint + q_dot * self.dt).tolist()

            # Calculate Forward Kinematics for the PREDICTED position
            T_next = self.robot.fkine(next_joint)
            x, y, z = T_next.t[0], T_next.t[1], T_next.t[2]

            # Check Distance from shoulder (z=0.2)
            z_offset = z - 0.2
            dist_sq = x**2 + y**2 + z_offset**2

            # Limits (0.03m to 0.53m)
            if not (0.03**2 <= dist_sq <= 0.53**2):
                self.get_logger().warn("WORKSPACE LIMIT REACHED! Stopping.")
                # Do not update joint
            else:
                # Safe to move
                self.joint = next_joint

        # --- MODE 3: TRAJECTORY ---
        if self.mode == 3:
            if self.traj_index < len(self.traj_q):
                self.joint = self.traj_q[self.traj_index].tolist()
                self.traj_index += 1
            if self.count >= 1000:
                self.call_random_service()
                self.count = 0
            else:
                self.get_logger().info("Target Reached.")
                pass
            self.count += 1

        # Publish
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint
        self.joint_pub.publish(msg)
        self.publish_fk()

    # --- CALLBACKS ---
    def cmd_vel_callback(self, msg):
        self.current_vel[0] = msg.linear.x
        self.current_vel[1] = msg.linear.y
        self.current_vel[2] = msg.linear.z

    def frame_callback(self, msg):
        self.control_frame = msg.data

    def mode_callback(self, request, response):
        if request.data == 1:
            q_sol = self.IK(request.position.x, request.position.y, request.position.z)
            if q_sol is not None:
                self.joint = q_sol.tolist()
                self.mode = 0
                response.success = True
            else:
                response.success = False
        elif request.data == 2:
            self.mode = 2
            self.get_logger().info("Switched to Teleoperation Mode")
            response.success = True
        elif request.data == 3:
            self.call_random_service()
            response.success = True
        return response

    # --- UTILS ---
    def call_random_service(self):
        if not self.cli_random.service_is_ready():
            return
        self.req_random.data = 1
        future = self.cli_random.call_async(self.req_random)
        future.add_done_callback(self.random_response_callback)

    def random_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                q_sol = self.IK(
                    response.position.x, response.position.y, response.position.z
                )
                if q_sol is not None:
                    self.start_trajectory(q_sol)
        except Exception as e:
            pass

    def start_trajectory(self, q_target):
        t_array = np.linspace(0, 1.0, 100)
        traj = rtb.jtraj(self.joint, q_target, t_array)
        self.traj_q = traj.q
        self.traj_index = 0
        self.mode = 3

    def IK(self, x, y, z):
        z_offset = z - 0.2
        distance_squared = x**2 + y**2 + z_offset**2
        if not (0.03**2 <= distance_squared <= 0.53**2):
            return None
        target_pose = SE3(x, y, z)
        ik_result = self.robot.ikine_LM(
            target_pose, mask=[1, 1, 1, 0, 0, 0], q0=[0, 0, 0]
        )
        return ik_result.q if ik_result.success else None

    def publish_fk(self):
        T_end_effector = self.robot.fkine(self.joint)
        fk_msg = PoseStamped()
        fk_msg.header.stamp = self.get_clock().now().to_msg()
        fk_msg.header.frame_id = "link_0"
        fk_msg.pose.position.x = T_end_effector.t[0]
        fk_msg.pose.position.y = T_end_effector.t[1]
        fk_msg.pose.position.z = T_end_effector.t[2]
        quat = UnitQuaternion(T_end_effector.R)
        fk_msg.pose.orientation.w = float(quat.s)
        fk_msg.pose.orientation.x = float(quat.v[0])
        fk_msg.pose.orientation.y = float(quat.v[1])
        fk_msg.pose.orientation.z = float(quat.v[2])
        self.fk_pub.publish(fk_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()