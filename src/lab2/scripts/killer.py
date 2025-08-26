#!/usr/bin/python3

from lab2.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose 
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty
import numpy as np
import math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from turtlesim.srv import Kill , Spawn

class DummyNode(Node):
    def __init__(self):
        super().__init__("killer_node")
        self.publisher = self.create_publisher(Twist, "/killer/cmd_vel", 10)
        self.spawn_turtle_client = self.create_client(Spawn, "/spawn_turtle")
        self.kill_turtle_client = self.create_client(Kill, "/remove_turtle")
        self.spawn_turtle_client.call_async(Spawn.Request(name="killer", x=1.0, y=1.0, theta=0.0))
        self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.create_subscription(Pose, "/killer/pose", self.killer_pose_callback, 10)
        self.create_subscription(Bool, "/hunt", self.hunt_callback, 10)
        self.create_timer(0.01, self.timer_callback)

        self.killer_pose = np.array([0.0, 0.0, 0.0])
        self.eater_pose = np.array([0.0, 0.0])
        self.kp_v = 5.0
        self.kp_omega = 20.0
        self.hunt = False

    def timer_callback(self):
        if self.hunt:
            self.control(self.eater_pose[0], self.eater_pose[1])
            self.get_logger().info(f"Eater position: {self.eater_pose}")
            if abs(self.eater_pose[0] -self.killer_pose[0]) < 0.5 and abs(self.eater_pose[1] - self.killer_pose[1]) < 0.5:
                self.kill_turtle_client.call_async(Kill.Request(name="turtle1"))
                # self.get_logger().info(f"Eater position")

    def pose_callback(self, msg):
        self.eater_pose = np.array([msg.x, msg.y])

    def killer_pose_callback(self, msg):
        self.killer_pose = np.array([msg.x, msg.y, msg.theta])

    def hunt_callback(self, msg):
        if msg.data == True:
            self.hunt = True

    def control(self, x, y):
        delta_x = x - self.killer_pose[0]
        delta_y = y - self.killer_pose[1]
        d = math.sqrt(((delta_x**2) + (delta_y**2))) 

        pizza = math.atan2(delta_y, delta_x)
        theta = pizza - self.killer_pose[2]
        w = math.atan2(math.sin(theta), math.cos(theta))

        v = self.kp_v * d
        wz = self.kp_omega * w
        self.move_turtle(v, wz)

    def move_turtle(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
