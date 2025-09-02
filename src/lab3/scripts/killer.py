#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Kill, Spawn
from std_srvs.srv import Empty
from std_msgs.msg import Int64
import math
from custom_interfaces.srv import SetParam, SetMaxPizza


class KillerNode(Node):
    def __init__(self):
        super().__init__('killer_node')

        self.spawn_turtle_client = self.create_client(Spawn, "/spawn_turtle")

        self.pub_cmdvel = self.create_publisher(Twist, 'cmd_vel', 10) 
        self.create_subscription(Pose, 'pose', self.pose_callback, 10)

        self.declare_parameter("sampling_frequency", 100.0)
        self.declare_parameter("turtle1_name", "eater")
        self.sampling_frequency = self.get_parameter("sampling_frequency").value
        self.turtle1_name = self.get_parameter("turtle1_name").value

        self.create_subscription(Pose, f'/{self.turtle1_name}/pose', self.target_callback, 10)
        self.create_subscription(Int64, f'/{self.turtle1_name}/pizza_count', self.pizza_count_callback, 10)
        self.create_subscription(Int64, '/set_max_pizza', self.set_max_pizza_callback, 10)

        self.eat_pizza_client = self.create_client(Kill, '/remove_turtle')

        self.srv1 = self.create_service(
            SetMaxPizza, "set_max_pizza", self.set_max_pizza1_callback          
        )
        self.srv = self.create_service(
            SetParam, "set_killer_kp", self.set_param_callback
        )

        self.timer = self.create_timer(
            1.0 / self.sampling_frequency, self.timer_callback
        )


        self.current_target = None
        self.current_pose = [0.0, 0.0, 0.0]
        self.controller_enable = False
        self.pizza_cnt = 0
        self.max_pizza = 5
        self.spawn = False

    def set_param_callback(self, request, response):
        self.kp_linear = request.kp_linear
        self.kp_angular = request.kp_angular
        return response

    def set_max_pizza1_callback(self, request, response):
        self.max_pizza = request.max_pizza.data
        response.log.data = "True"
        return response

    def spawn_turtle_once(self):
        if self.spawn:
            return
        if not self.spawn_turtle_client.service_is_ready():
            return
        self.spawn_turtle("turtle2")
        self.spawn = True

    def spawn_turtle(self, name):
        request = Spawn.Request()
        request.x = 3.0
        request.y = 5.0
        request.theta = 0.0
        request.name = name

        self.spawn_turtle_client.call_async(request)

    def target_callback(self, msg: Pose):
        if self.pizza_cnt == self.max_pizza:
            self.current_target = [msg.x, msg.y]
            self.controller_enable = True

    def pose_callback(self, msg: Pose):
        self.current_pose[0] = msg.x
        self.current_pose[1] = msg.y
        self.current_pose[2] = msg.theta

    def pizza_count_callback(self, msg: Int64):
        self.pizza_cnt = msg.data

    def set_max_pizza_callback(self, msg : Int64):
        self.max_pizza = msg.data

    def kill_turtle(self, name : str):
        kill_request = Kill.Request()
        kill_request.name = name
        self.eat_pizza_client.call_async(kill_request)

    def cmd_vel(self, vx, w):
        cmd_vel = Twist()
        cmd_vel.linear.x = vx
        cmd_vel.angular.z = w
        self.pub_cmdvel.publish(cmd_vel)

    def timer_callback(self):
        # self.get_logger().info(f"max_pizza_killer: {self.turtle1_name}")
        if self.controller_enable:
            dx = self.current_target[0] - self.current_pose[0]
            dy = self.current_target[1] - self.current_pose[1]

            e_dis = math.hypot(dx, dy)
            e_ori = math.atan2(dy, dx) - self.current_pose[2]
            e_ori = math.atan2(math.sin(e_ori), math.cos(e_ori))

            u_dis = 2 * e_dis
            u_ori = 10 * e_ori

            if (abs(dx) < 0.1 and abs(dy) < 0.1):
                self.cmd_vel(0.0, 0.0)
                self.kill_turtle(self.turtle1_name)
                self.controller_enable = False
            else:
                self.cmd_vel(u_dis, u_ori)



def main(args=None):
    rclpy.init(args=args)
    node = KillerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
