#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped, Twist,PoseStamped
from std_msgs.msg import String,Header
from robot_interfaces.srv import Scheduler, RandomEndeffector, ControllerData
import numpy as np

# Inverse Kinematics
import roboticstoolbox as rtb
from math import pi
from spatialmath import SE3
from scipy.spatial.transform import Rotation as R  # Import scipy Rotation
from sensor_msgs.msg import JointState  # Import JointState message


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")

        # Set Callback Frequency
        self.declare_parameter("frequency", 100.0)
        self.frequency = (
            self.get_parameter("frequency").get_parameter_value().double_value
        )
        self.create_timer(1 / self.frequency, self.timer_callback)

        # Create controller Server
        self.controller_server = self.create_service(
            ControllerData, "controller_server", self.controller_server_callback
        )
        self.controller_state = "IDLE"

        # Create robot state subscriber and client
        self.current_state = "IDLE"
        self.create_subscription(
            String, "/current_state", self.current_state_callback, 10
        )
        self.scheduler_client = self.create_client(Scheduler, "robot_state_server")

        # Create cmd_vel subscriber
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.tele_x = 0
        self.tele_y = 0
        self.tele_z = 0

        # Create TF subscriber to pub topic /end_effector
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_frame = "end_effector"
        self.source_frame = "link_0"

        # pub target and end_effector
        self.target_pub = self.create_publisher(PoseStamped, "/target", 10)
        self.endeff_pub = self.create_publisher(PoseStamped, "/end_effector", 10)

        # Robot parameters
        self.kp = 1.0  # Proportional gain
        self.q = np.array([0.0, 0.0, 0.0])  # Initial joint angles
        self.r_max = 0.28 + 0.25
        self.r_min = 0.03
        self.l = 0.2
        self.ik_setpoint = [0, 0, 0]
        self.random_setpoint = [0, 0, 0]

        # Initialize JointState publisher
        self.joint_state_publisher = self.create_publisher(
            JointState, "joint_states", 10
        )
        self.joint_state = JointState()
        self.joint_state.header.frame_id = ""
        self.joint_state.name = ["joint_1", "joint_2", "joint_3"]
        self.joint_state.position = [0.0, 0.0, 0.0]

        # Define robot once in __init__
        self.robot = rtb.DHRobot(
            [
                rtb.RevoluteMDH(alpha=0.0, a=0.0, d=0.2, offset=0.0),
                rtb.RevoluteMDH(alpha=pi / 2, a=0.0, d=0.02, offset=0.0),
                rtb.RevoluteMDH(alpha=0.0, a=0.25, d=0.0, offset=0.0),
            ],
            tool=SE3.Tx(0.28),
            name="RRR_Robot",
        )
        self.publish_joint_state(np.array([0.0, 0.0, 0.0]))

        self.get_logger().info("controller_node has been started.")

    def cmd_vel_callback(self, msg: Twist):
        self.tele_x = msg.linear.x
        self.tele_y = msg.linear.y
        self.tele_z = msg.linear.z
        if self.controller_state == "TELEOP_G":
            continue_control = self.control_vel("TELEOP_G")
        elif self.controller_state == "TELEOP_F":
            continue_control = self.control_vel("TELEOP_F")

    def req_scheduler(self, state):
        self.get_logger().info("Request robot_scheduler node")
        state_request = Scheduler.Request()
        state_request.state.data = str(state)
        future = self.scheduler_client.call_async(state_request)

    def controller_server_callback(
        self, request: ControllerData.Request, response: ControllerData.Response
    ):
        self.get_logger().info(
            f"Request to change controller state from {self.controller_state} to: {request.mode.data}"
        )
        self.controller_state = request.mode.data

        if self.controller_state == "AUTO":
            self.random_setpoint = [
                float(request.position.x),
                float(request.position.y),
                float(request.position.z),
            ]

        elif self.controller_state == "IK":
            self.ik_setpoint = [
                float(request.position.x),
                float(request.position.y),
                float(request.position.z),
            ]

        elif self.controller_state == "TELEOP_F":
            pass

        elif self.controller_state == "TELEOP_G":
            pass

        response.inprogress = True
        return response

    def current_state_callback(self, msg: String):
        self.current_state = msg.data

    def publish_joint_state(self, positions):
        if len(positions) != len(self.joint_state.name):
            self.get_logger().error(
                f"Number of positions ({len(positions)}) does not match number of joints ({len(self.joint_state.name)})."
            )
            return

        self.joint_state.header.stamp = self.get_clock().now().to_msg()

        # Update joint positions
        try:
            self.joint_state.position = positions.tolist()  # Ensure it's a list
        except:
            self.joint_state.position = positions  # Ensure it's a list

        self.joint_state_publisher.publish(self.joint_state)
        self.get_logger().info("Published JointState message.")

    def control_vel(self, mode):
        try:
            # Using the robot defined in __init__
            robot = self.robot

            result = self.get_transform()
            if result is None:
                self.get_logger().error("get_transform returned None.")
                return False
            p_now, r_e = result
            if p_now is None:
                self.get_logger().error(
                    "Current end-effector position could not be retrieved."
                )
                return False

            # Compute desired end-effector velocity (p_dot)
            if mode == "TELEOP_G":
                p_dot = np.array([self.tele_x, self.tele_y, self.tele_z])
            elif mode == "TELEOP_F":
                p_dot = r_e @ np.array([self.tele_x, self.tele_y, self.tele_z])
            else:
                self.get_logger().warning(f"TELEOP incorrect mode")
                return False
            # Compute Jacobian
            J = robot.jacob0(self.q)
            J_pos = J[0:3, :]  # Position part of the Jacobian

            # Singularity checking by determinant
            det_J = np.linalg.det(J_pos)
            det_threshold = 1e-100

            if abs(det_J) < det_threshold:
                self.get_logger().warning(
                    f"Near singularity detected. Determinant of J: {det_J:.20f}. Stopping movement."
                )
                return False

            # Compute joint velocities (q_dot)
            q_dot = np.linalg.pinv(J_pos) @ p_dot

            # Update joint angles
            self.q = self.q + q_dot * (1.0 / self.frequency)
            self.get_logger().info(f"q : {self.q}")

            # Publish joint states
            self.publish_joint_state(self.q)

        except Exception as e:
            self.get_logger().error(f"Failed in control_vel: {e}")
            return False

    def control_to_pos(self, p_set):
        try:
            # Using the robot defined in __init__
            robot = self.robot

            p_setpoint = np.array(p_set)

            result = self.get_transform()
            if result is None:
                self.get_logger().error("get_transform returned None.")
                return False
            p_now, r_e = result
            if p_now is None:
                self.get_logger().error(
                    "Current end-effector position could not be retrieved."
                )
                return False

            error = p_setpoint - p_now
            # Compute desired end-effector velocity (p_dot)
            p_dot = self.kp * error
            # Compute Jacobian
            J = robot.jacob0(self.q)
            J_pos = J[0:3, :]  # Position part of the Jacobian

            # Singularity checking by determinant
            det_J = np.linalg.det(J_pos)
            det_threshold = 1e-100

            if abs(det_J) < det_threshold:
                self.get_logger().warning(
                    f"Near singularity detected. Determinant of J: {det_J:.20f}. Stopping movement."
                )
                self.req_scheduler("IDLE")
                self.controller_state = "IDLE"
                return False

            # Compute joint velocities (q_dot)
            q_dot = np.linalg.pinv(J_pos) @ p_dot

            # Update joint angles
            self.q = self.q + q_dot * (1.0 / self.frequency)
            self.get_logger().info(f"q : {self.q}")

            # Publish joint states
            self.publish_joint_state(self.q)

            if np.linalg.norm(error) <= 0.001:
                self.get_logger().info("Target position reached.")
                self.req_scheduler("IDLE")
                self.controller_state = "IDLE"
                return False
            else:
                return True

        except Exception as e:
            self.get_logger().error(f"Failed in control_to_pos: {e}")
            return False

    def get_transform(self):
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                self.source_frame, self.target_frame, now
            )
            position = transform.transform.translation
            orientation = transform.transform.rotation
            x = position.x
            y = position.y
            z = position.z

            self.get_logger().info(f"End Effector Position: {x}, {y}, {z}")
            # self.get_logger().info(f"End Effector Orientation: {orientation.x}, {orientation.y}, {orientation.z}, {orientation.w}")
            # Extract quaternion components
            qx = orientation.x
            qy = orientation.y
            qz = orientation.z
            qw = orientation.w

            rotation = R.from_quat([qx, qy, qz, qw])  # Note: scipy expects [x, y, z, w]
            rotation_matrix = rotation.as_matrix()  # Get 3x3 rotation matrix

            return (
                np.array([x, y, z]),
                rotation_matrix,
            )  # Return the position as an array

        except Exception as e:
            self.get_logger().error(f"Failed to get transform: {e}")
            self.publish_joint_state(np.array([0.0, 0.0, 0.0]))  # Reborn
            return None, None  # Ensure a tuple is returned

    def inverse_kinematic(self, x, y, z):
        distance_squared = x**2 + y**2 + (z - 0.2) ** 2
        if self.r_min**2 <= distance_squared <= self.r_max**2:

            robot = self.robot  # Use the robot defined in __init__

            T_Position = SE3(x, y, z)
            ik_solution = robot.ikine_LM(
                T_Position, mask=[1, 1, 1, 0, 0, 0], joint_limits=False, q0=self.q
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

    def rviz_pub(self,pos):
        target = PoseStamped()
        target.header = Header()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = 'link_0'
        target.pose.position.x = float(pos[0])
        target.pose.position.y = float(pos[1])
        target.pose.position.z = float(pos[2])

        self.target_pub.publish(target)

        result = self.get_transform()
        if result is None:
            self.get_logger().error("get_transform returned None.")
            return False
        p_now, r_e = result
        if p_now is None:
            self.get_logger().error(
                "Current end-effector position could not be retrieved."
            )
            return False

        endeff = PoseStamped()
        endeff.header = Header()
        endeff.header.stamp = self.get_clock().now().to_msg()
        endeff.header.frame_id = "link_0"
        endeff.pose.position.x = float(p_now[0])
        endeff.pose.position.y = float(p_now[1])
        endeff.pose.position.z = float(p_now[2])

        self.endeff_pub.publish(endeff)

    def timer_callback(self):
        try:
            if self.controller_state == "AUTO":
                continue_control = self.control_to_pos(self.random_setpoint)
                self.rviz_pub(self.random_setpoint)
                if not continue_control:
                    self.get_logger().info("Switching to IDLE state.")
                    self.req_scheduler("IDLE")
                    self.controller_state = "IDLE"

            elif self.controller_state == "IK":
                continue_control = self.control_to_pos(self.ik_setpoint)
                self.rviz_pub(self.ik_setpoint)
                if not continue_control:
                    self.get_logger().info("Switching to IDLE state.")
                    self.req_scheduler("IDLE")
                    self.controller_state = "IDLE"

        except Exception as e:
            self.get_logger().error(f"Failed in timer_callback: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
