#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from turtlesim_plus_interfaces.srv import GivePosition
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Point
from std_srvs.srv import Empty
from std_msgs.msg import Int64
from turtlesim.srv import Kill, Spawn
from rcl_interfaces.msg import SetParametersResult
import math
import yaml
import itertools
import sys

class DummyNode(Node):
    def __init__(self):
        super().__init__('teleoper_node')

        self.declare_parameter("sampling_frequency", 100.0)
        self.declare_parameter("kp_linear", 2.0)
        self.declare_parameter("kp_angular", 10.0)
        self.declare_parameter("eraser_kp_linear", 2.0)
        self.declare_parameter("eraser_kp_angular", 10.0)
        self.declare_parameter("max_pizza", 12)

        self.sampling_frequency = self.get_parameter("sampling_frequency").value
        self.kp_linear = self.get_parameter("kp_linear").value  
        self.kp_angular = self.get_parameter("kp_angular").value
        self.max_pizza = self.get_parameter("max_pizza").value
        self.eraser_kp_linear = self.get_parameter("eraser_kp_linear").value
        self.eraser_kp_angular = self.get_parameter("eraser_kp_angular").value
        
        # --- BONUS 2: Add Callback for Dynamic Parameters ---
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.pub_cmdvel = self.create_publisher(Twist, "teleoper/cmd_vel", 10)
        self.eraser_pub_cmdvel = self.create_publisher(Twist, "eraser/cmd_vel", 10)

        self.create_subscription(Pose, "teleoper/pose", self.pose_callback, 10)
        self.create_subscription(Pose, "/eraser_spawn_pose", self.eraser_spawn_pose_callback, 10)
        self.create_subscription(Pose, "eraser/pose", self.eraser_pose_callback, 10)
        self.create_subscription(Int64, "teleoper/pizza_count", self.eat_pizza_count_callback, 10)
        self.create_subscription(Int64, "eraser/pizza_count", self.eraser_eat_pizza_count_callback, 10)
        self.create_subscription(Point, "mouse_position", self.mouse_position_callback, 10)
        
        self.pub_max_pizza = self.create_publisher(Int64, "set_max_pizza", 10) 

        self.srv = self.create_service(AddTwoInts, 'order', self.order_callback)
        self.order_client = self.create_client(AddTwoInts, "order_to_copy") 
        
        self.spawn_pizza_client = self.create_client(GivePosition, "spawn_pizza")
        self.eat_pizza_client = self.create_client(Empty, "teleoper/eat")
        self.eraser_eat_pizza_client = self.create_client(Empty, "eraser/eat")
        self.spawn_turtle_client = self.create_client(Spawn, "spawn_turtle")
        self.kill_turtle_client = self.create_client(Kill, "remove_turtle")
        
        self.timer = self.create_timer(
            1.0 / self.sampling_frequency, self.timer_callback
        )

        self.current_pose = [0.0, 0.0, 0.0] 
        self.target_queue = []
        self.target_saved = []
        self.pizza_cnt = 0
        self.is_eat_all = False
        self.noteatyet = False
        self.number_of_pizza = 0
        self.current_target = None
        self.controller_enable = False
        self.get_logger().info('Teleoper Node Started')
        self.num_of_saved = 0
        self.eraser_target = []
        self.eraser_controller_enable = False
        self.eraser_current_target = None
        self.eraser_is_eat_all = False
        self.eraser_noteatyet = False
        self.eraser_spawn_pos = [0.0, 0.0]
        self.eraser_current_pose = [0.0, 0.0, 0.0]
        self.eraser_number_of_pizza = 0
        self.end=False
        self.first_time=True

    def parameters_callback(self, params):
        for param in params:
            if param.name == "kp_linear":
                self.kp_linear = param.value
                self.get_logger().info(f"Updated kp_linear: {self.kp_linear}")
            elif param.name == "kp_angular":
                self.kp_angular = param.value
                self.get_logger().info(f"Updated kp_angular: {self.kp_angular}")
            elif param.name == "max_pizza":
                self.max_pizza = param.value
                self.get_logger().info(f"Updated max_pizza: {self.max_pizza}")
            elif param.name == "eraser_kp_linear":
                self.eraser_kp_linear = param.value
            elif param.name == "eraser_kp_angular":
                self.eraser_kp_angular = param.value
        return SetParametersResult(successful=True)

    def order_callback(self, request, response):
        if request.a == 1:
            if self.current_pose[0]==0.0 :
                self.current_pose[0]=0.00001
            if self.current_pose[1]==0.0 :
                self.current_pose[1]=0.00001
            point = [self.current_pose[0], self.current_pose[1]]
            if self.pizza_cnt < self.max_pizza:
                self.target_queue.append(point)
                self.pizza_cnt += 1
                self.spawn_pizza(point)
        elif (request.a == 2):
            self.target_saved.append(self.target_queue)
            self.target_queue = []
            self.num_of_saved += 1
            if (self.num_of_saved == 4):
                yaml.dump(self.target_saved, open("targets.yaml", "w"))
                self.get_logger().info('Saved to targets.yaml')
                order_request = AddTwoInts.Request()
                order_request.a = 1
                order_request.b = 0
                self.order_client.call_async(order_request)
                self.max_pizza=self.pizza_cnt

        elif (request.a == 3 and self.target_queue!=[] ):
            self.controller_enable = True
            self.number_of_pizza = len(self.target_queue)
            self.pizza_cnt -= self.number_of_pizza

        elif (request.a == 4 and self.first_time):
            self.first_time=False
            self.spawn_turtle("eraser")
            self.eraser_target = list(itertools.chain.from_iterable(self.target_saved))
            if self.eraser_target!=[]:
                self.eraser_controller_enable = True
                self.eraser_number_of_pizza = len(self.eraser_target)
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response

    def spawn_turtle(self, name):
        request = Spawn.Request()
        request.x = self.eraser_spawn_pos[0]
        request.y = self.eraser_spawn_pos[1]
        request.theta = 0.0
        request.name = name
        self.spawn_turtle_client.call_async(request)

    def kill_turtle(self, name: str):
        kill_request = Kill.Request()
        kill_request.name = name
        self.kill_turtle_client.call_async(kill_request)

    def spawn_pizza(self, position):
        position_request = GivePosition.Request()
        position_request.x = position[0]
        position_request.y = position[1]
        self.spawn_pizza_client.call_async(position_request)

    def pose_callback(self, msg):
        self.current_pose[0] = msg.x
        self.current_pose[1] = msg.y
        self.current_pose[2] = msg.theta

    def eraser_pose_callback(self, msg):
        self.eraser_current_pose[0] = msg.x
        self.eraser_current_pose[1] = msg.y
        self.eraser_current_pose[2] = msg.theta

    def eat_pizza_count_callback(self, msg: Int64):
        self.is_eat_all = msg.data == self.number_of_pizza
        if self.is_eat_all and self.controller_enable:
            self.controller_enable = False
            self.target_queue = []
            self.number_of_pizza = 0

    def eraser_eat_pizza_count_callback(self, msg: Int64):
        self.eraser_is_eat_all = msg.data == self.eraser_number_of_pizza

    def eat_pizza(self):
        eat_request = Empty.Request()
        self.eat_pizza_client.call_async(eat_request)

    def eraser_eat_pizza(self):
        eat_request = Empty.Request()
        self.eraser_eat_pizza_client.call_async(eat_request)

    def timer_callback(self):
        self.control_to_target()
        if self.num_of_saved >= 4:
            self.controller_enable = False
        if self.eraser_controller_enable :
            self.eraser_control_to_target()
        
        # --- Shutdown Logic ---
        if self.end:
            self.end = False
            self.kill_turtle("eraser")
            self.kill_turtle("teleoper")
            self.eraser_controller_enable = False
            self.eraser_target = []
            self.eraser_number_of_pizza = 0
            
            order_request = AddTwoInts.Request()
            order_request.a = 2
            order_request.b = 0
            self.order_client.call_async(order_request)
            
            self.get_logger().info("Mission Complete. Shutting down Teleoper node.")
            
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)


    def cmd_vel(self, vx, w):
        cmd_vel = Twist()
        cmd_vel.linear.x = vx
        cmd_vel.angular.z = w
        self.pub_cmdvel.publish(cmd_vel)

    def eraser_cmd_vel(self, vx, w):
        cmd_vel = Twist()
        cmd_vel.linear.x = vx
        cmd_vel.angular.z = w
        self.eraser_pub_cmdvel.publish(cmd_vel)

    def mouse_position_callback(self, msg: Point):
        self.get_logger().info(f"Mouse Position: x={msg.x}, y={msg.y}")

    def control_to_target(self):
        a=Int64()
        a.data=self.max_pizza
        self.pub_max_pizza.publish(a)
        
        if len(self.target_queue) > 0 and self.controller_enable and self.noteatyet == False:
            self.current_target = self.target_queue.pop(0)
            self.noteatyet = True

        if self.controller_enable and self.current_target:
            dx = self.current_target[0] - self.current_pose[0]
            dy = self.current_target[1] - self.current_pose[1]

            e_dis = math.hypot(dx, dy)
            e_ori = math.atan2(dy, dx) - self.current_pose[2]
            e_ori = math.atan2(math.sin(e_ori), math.cos(e_ori))

            u_dis = self.kp_linear * e_dis
            u_ori = self.kp_angular * e_ori

            if abs(dx) < 0.1 and abs(dy) < 0.1:
                self.cmd_vel(0.0, 0.0)
                if not self.is_eat_all:
                    self.eat_pizza()
                    self.noteatyet = False
            else:
                self.cmd_vel(u_dis, u_ori)

    def eraser_control_to_target(self):
        if self.eraser_is_eat_all:
            # Set target to teleoper
            self.eraser_current_target = [self.current_pose[0],self.current_pose[1]]
            self.eraser_noteatyet = True
        
        if (
            len(self.eraser_target) > 0
            and self.eraser_controller_enable
            and self.eraser_noteatyet == False
        ):
            self.eraser_current_target = self.eraser_target.pop(0)
            self.eraser_noteatyet = True
            
        if self.eraser_controller_enable and self.eraser_current_target:
            dx = self.eraser_current_target[0] - self.eraser_current_pose[0]
            dy = self.eraser_current_target[1] - self.eraser_current_pose[1]

            e_dis = math.hypot(dx, dy)
            e_ori = math.atan2(dy, dx) - self.eraser_current_pose[2]
            e_ori = math.atan2(math.sin(e_ori), math.cos(e_ori))

            u_dis = self.eraser_kp_linear * e_dis
            u_ori = self.eraser_kp_angular * e_ori

            # Check if close to target
            if abs(dx) < 0.1 and abs(dy) < 0.1:
                self.eraser_cmd_vel(0.0, 0.0)
                if not self.eraser_is_eat_all:
                    self.eraser_eat_pizza()
                    self.eraser_noteatyet = False
                    
                elif self.eraser_is_eat_all :
                    # This block handles the Final Kill logic
                    # FIX: Increased tolerance check implicitly by being in the <0.1 block
                    # If we are here, we are close enough to the Teleoper (since current_target is teleoper)
                    self.get_logger().info('Eraser finished eating all pizzas and will be removed.')
                    self.kill_turtle("teleoper")
                    self.eraser_noteatyet = False
                    self.end = True # Triggers shutdown in timer_callback
            else:
                self.eraser_cmd_vel(u_dis, u_ori)

    def set_max_pizza1_callback(self, request, response):
        self.max_pizza = request.max_pizza.data
        response.log.data = "True"
        return response
    
    def eraser_spawn_pose_callback(self, msg):
        self.eraser_spawn_pos[0] = msg.x
        self.eraser_spawn_pos[1] = msg.y
        

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    
    # FIX: Robustly handle SystemExit to ensure clean shutdown
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("teleoper_node").info("Node stopped cleanly.")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()