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

class DummyNode(Node):
    def __init__(self):
        super().__init__('eater_node')

        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.hunt = self.create_publisher(Bool, "/hunt", 10)
        self.create_subscription(Point, '/mouse_position', self.mouse_position_callback, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_pose_callback, 10)
        self.spawn_pizza_client = self.create_client(GivePosition, '/spawn_pizza')
        self.pizza_eat_client = self.create_client(Empty, '/turtle1/eat')
        self.create_timer(0.01, self.timer_callback)
        self.turtle_pose = np.array([0.0, 0.0, 0.0])
        self.mouse_pose = np.array([0.0, 0.0])
        self.kp_v = 5.0
        self.kp_omega = 20.0
        self.num_pizza = []
        self.count = 0
        self.pizza_count = 0
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal = 0
        self.true=Bool()
        self.true.data=True

    def timer_callback(self):
        # self.get_logger().info(f"pizza={self.num_pizza}")
        if self.goal == 1:
            self.control(self.goal_x, self.goal_y)
        else:
            if len(self.num_pizza) <= 5 and self.count <= self.pizza_count:
                if self.count < self.pizza_count:
                    if len(self.num_pizza) == 0:
                        return
                    else:
                        self.control(
                            self.num_pizza[self.count][0], self.num_pizza[self.count][1]
                        )
                elif self.count == self.pizza_count:    
                    self.move_turtle(0.0, 0.0)
            else:
                self.control(self.mouse_pose[0], self.mouse_pose[1])
                self.hunt.publish(self.true)

    def mouse_position_callback(self, msg):
        call = GivePosition.Request()
        call.x = msg.x
        call.y = msg.y
        self.pizza_count += 1
        self.mouse_pose = np.array([msg.x, msg.y])
        self.num_pizza.append((self.mouse_pose[0], self.mouse_pose[1]))
        if (len(self.num_pizza)<=5 ):
            self.spawn_pizza_client.call_async(call)

    def pose_callback(self, msg):
        self.turtle_pose = np.array([msg.x, msg.y, msg.theta])
        # self.get_logger().info(f"Pose: x={msg.x}, y={msg.y}, theta={msg.theta}")

    def move_turtle(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher.publish(twist)

    def eat_pizza(self):
        call=Empty.Request()
        self.get_logger().info(f"Pizza eaten: {self.count}")
        self.pizza_eat_client.call_async(call)

    def control(self,x,y):
        delta_x = x - self.turtle_pose[0]
        delta_y = y - self.turtle_pose[1]
        d = math.sqrt(((delta_x**2) + (delta_y**2))) - 0.6

        pizza = math.atan2(delta_y, delta_x)
        theta = pizza - self.turtle_pose[2]
        w = math.atan2(math.sin(theta), math.cos(theta))

        v = self.kp_v * d
        wz = self.kp_omega * w
        self.move_turtle(v, wz)
        if self.goal == 1:
            if d < 0.5 and abs(w) < 0.1:
                self.goal = 0
        else:
            if d < 0.5 and abs(w) < 0.1:
                self.eat_pizza()
                self.count += 1

    def goal_pose_callback(self, msg):
        self.goal = 1
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y


def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
