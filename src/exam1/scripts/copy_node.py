#!/usr/bin/python3

# from exam1.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from turtlesim_plus_interfaces.srv import GivePosition
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Point, PoseStamped
from std_srvs.srv import Empty
from std_msgs.msg import Int64, Float64
from turtlesim.srv import Kill, Spawn
from custom_interfaces.srv import SetParam, SetMaxPizza
import math
import yaml
import sys

class DummyNode(Node):
    def __init__(self):
        super().__init__("dummy_node")
        self.declare_parameter("sampling_frequency", 100.0)
        self.declare_parameter("Foxy_kp_linear", 2.0)
        self.declare_parameter("Foxy_kp_angular", 10.0)
        self.declare_parameter("Noetic_kp_linear", 2.0)
        self.declare_parameter("Noetic_kp_angular", 10.0)
        self.declare_parameter("Humble_kp_linear", 2.0)
        self.declare_parameter("Humble_kp_angular", 10.0)
        self.declare_parameter("Iron_kp_linear", 2.0)
        self.declare_parameter("Iron_kp_angular", 10.0)
        self.declare_parameter("eraser_kp_linear", 2.0)
        self.declare_parameter("eraser_kp_angular", 10.0)

        self.sampling_frequency = self.get_parameter("sampling_frequency").value
        self.kp_linear = [0,0,0,0]
        self.kp_angular = [0,0,0,0]
        self.kp_linear[0] = self.get_parameter("Foxy_kp_linear").value
        self.kp_angular[0] = self.get_parameter("Foxy_kp_angular").value
        self.kp_linear[1] = self.get_parameter("Noetic_kp_linear").value
        self.kp_angular[1] = self.get_parameter("Noetic_kp_angular").value
        self.kp_linear[2] = self.get_parameter("Humble_kp_linear").value
        self.kp_angular[2] = self.get_parameter("Humble_kp_angular").value
        self.kp_linear[3] = self.get_parameter("Iron_kp_linear").value
        self.kp_angular[3] = self.get_parameter("Iron_kp_angular").value
        self.eraser_kp_linear = self.get_parameter("eraser_kp_linear").value
        self.eraser_kp_angular = self.get_parameter("eraser_kp_angular").value

        # FIX: Relative paths
        self.pub_cmdvel1 = self.create_publisher(Twist, "Foxy/cmd_vel", 10)
        self.pub_cmdvel2 = self.create_publisher(Twist, "Noetic/cmd_vel", 10)
        self.pub_cmdvel3 = self.create_publisher(Twist, "Humble/cmd_vel", 10)
        self.pub_cmdvel4 = self.create_publisher(Twist, "Iron/cmd_vel", 10)
        self.eraser_pub_cmdvel = self.create_publisher(Twist, "eraser/cmd_vel", 10)
        
        self.eraser_spawn_pose_pub = self.create_publisher(
            Pose, "/eraser_spawn_pose", 10 # Global topic
        )

        self.create_subscription(Pose, "Foxy/pose", self.pose1_callback, 10)
        self.create_subscription(Pose, "Noetic/pose", self.pose2_callback, 10)
        self.create_subscription(Pose, "Humble/pose", self.pose3_callback, 10)
        self.create_subscription(Pose, "Iron/pose", self.pose4_callback, 10)
        self.create_subscription(Pose, "eraser/pose", self.eraser_pose_callback, 10)

        self.create_subscription(
            Int64, "eraser/pizza_count", self.eat_pizza_count_callback, 10
        )

        self.create_subscription(
            Point, "mouse_position", self.mouse_position_callback, 10
        )

        # Services
        self.srv = self.create_service(AddTwoInts, "order_to_copy", self.order_callback)
        # 'order' will be remapped to '/order' (Global, where Teleoper is)
        self.order_client = self.create_client(AddTwoInts, "order") 
        
        self.spawn_pizza_client = self.create_client(GivePosition, "spawn_pizza")
        self.spawn_turtle_client = self.create_client(Spawn, "spawn_turtle")
        self.kill_turtle_client = self.create_client(Kill, "remove_turtle")
        self.eat_pizza_client = self.create_client(Empty, "eraser/eat")

        self.timer = self.create_timer(
            1.0 / self.sampling_frequency, self.timer_callback
        )
        self.current_target = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
        self.current_pose = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]] 
        self.target_queue = [[], [], [], []]  
        self.start = False
        self.last_click= False
        self.noteatyet = [False, False, False, False]
        self.number_of_pizza = 0
        self.is_eat_all = [False, False, False, False]
        self.controller_enable = [False, False, False, False]

        self.pizza_cnt = 0
        self.mouse_enabled = False
        self.pizza_spawned_at_target = [False, False, False, False]

        self.eraser_target = []
        self.eraser_controller_enable = False

        self.eraser_current_target = None
        self.eraser_is_eat_all = False
        self.eraser_noteatyet = False
        self.eraser_spawn_pos = [0.0, 0.0]
        self.eraser_current_pose = [0.0, 0.0, 0.0]
        
        self.first_time = True

    def timer_callback(self):
        if (self.start):
            self.control_to_target(1)
            self.control_to_target(2)
            self.control_to_target(3)
            self.control_to_target(4)
        if (self.target_queue[0] == [] and self.target_queue[1] == [] and self.target_queue[2] == [] and self.target_queue[3] == [] and not self.last_click):
            self.mouse_enabled = True
        if (self.eraser_controller_enable):
            self.eraser_control_to_target()
        if self.eraser_is_eat_all:
            self.eraser_is_eat_all = False
            self.kill_turtle("eraser")
            self.get_logger().info("Eraser finished. Shutting down copy node.")
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)


    def pose1_callback(self, msg):
        self.current_pose[0] = [msg.x, msg.y, msg.theta]

    def pose2_callback(self, msg):
        self.current_pose[1] = [msg.x, msg.y, msg.theta]

    def pose3_callback(self, msg):
        self.current_pose[2] = [msg.x, msg.y, msg.theta]

    def pose4_callback(self, msg):
        self.current_pose[3] = [msg.x, msg.y, msg.theta]

    def eraser_pose_callback(self, msg):
        self.eraser_current_pose = [msg.x, msg.y, msg.theta]

    def eat_pizza_count_callback(self, msg: Int64):
        # self.get_logger().info(f"eraser all eaten:{self.eraser_is_eat_all}")
        self.eraser_is_eat_all = msg.data == self.number_of_pizza
        if self.eraser_is_eat_all and self.eraser_controller_enable:
            self.eraser_controller_enable = False

    def order_callback(self, request, response):
        if (request.a == 1):
            self.controller_enable[0] = True
            self.controller_enable[1] = True
            self.controller_enable[2] = True
            self.controller_enable[3] = True
            self.load_targets_from_yaml()
            self.start = True
            self.get_logger().info(f"Loaded from targets.yaml: {self.target_queue}")
        if (request.a == 2):
            self.spawn_turtle("eraser")
            self.eraser_controller_enable = True
            self.number_of_pizza = self.pizza_cnt
            # Logic Note: Ideally eraser chases them, but code kills instantly
            self.kill_turtle("Foxy")
            self.kill_turtle("Noetic")
            self.kill_turtle("Humble")
            self.kill_turtle("Iron")
        return response

    def control_to_target(self, number):
        number = number - 1

        if (
            len(self.target_queue[number]) > 0
            and self.controller_enable[number]
            and self.noteatyet[number] == False
        ):
            self.current_target[number] = self.target_queue[number].pop(0) 
            self.noteatyet[number] = True
            self.pizza_spawned_at_target[number] = False

        if self.controller_enable[number] :
            dx = self.current_target[number][0] - self.current_pose[number][0]
            dy = self.current_target[number][1] - self.current_pose[number][1]

            e_dis = math.hypot(dx, dy)
            e_ori = math.atan2(dy, dx) - self.current_pose[number][2]
            e_ori = math.atan2(math.sin(e_ori), math.cos(e_ori))

            u_dis = self.kp_linear[number] * e_dis
            u_ori = self.kp_angular[number] * e_ori

            if abs(dx) < 0.1 and abs(dy) < 0.1:
                self.cmd_vel(0.0, 0.0,number+1)
                if (
                    not self.is_eat_all[number]
                    and not self.pizza_spawned_at_target[number]
                ):
                    self.get_logger().info(f"current={self.current_target[number]}")
                    if not self.last_click :
                        point = [
                            self.current_pose[number][0],
                            self.current_pose[number][1],
                        ]
                        if self.current_target[number] != [0.0, 0.0]:
                            self.spawn_pizza(point)
                            self.eraser_target.append(point)
                            self.pizza_cnt += 1
                        self.pizza_spawned_at_target[number] = True
                    if self.last_click:
                        self.controller_enable[number] = False
                        if (self.controller_enable == [False, False, False, False] and self.last_click and self.first_time):
                            order_request = AddTwoInts.Request()
                            order_request.a = 4
                            order_request.b = 0
                            self.order_client.call_async(order_request)
                            self.first_time = False

                    self.noteatyet[number] = False
            else:
                self.cmd_vel(u_dis, u_ori, number + 1)

    def cmd_vel(self, vx, w, number):
        cmd_vel = Twist()
        cmd_vel.linear.x = vx
        cmd_vel.angular.z = w
        if number == 1:
            self.pub_cmdvel1.publish(cmd_vel)
        elif number == 2:
            self.pub_cmdvel2.publish(cmd_vel)
        elif number == 3:
            self.pub_cmdvel3.publish(cmd_vel)
        elif number == 4:
            self.pub_cmdvel4.publish(cmd_vel)

    def eraser_cmd_vel(self, vx, w):   
        cmd_vel = Twist()
        cmd_vel.linear.x = vx
        cmd_vel.angular.z = w
        self.eraser_pub_cmdvel.publish(cmd_vel)

    def mouse_position_callback(self, msg: Point):
        if self.mouse_enabled:
            point = [msg.x, msg.y]
            self.target_queue[0].append(point)
            self.target_queue[1].append(point)
            self.target_queue[2].append(point)
            self.target_queue[3].append(point)
            self.eraser_spawn_pos=point
            self.controller_enable[0] = True
            self.controller_enable[1] = True
            self.controller_enable[2] = True
            self.controller_enable[3] = True
            self.mouse_enabled = False
            self.last_click= True
            meg = Pose()
            meg.x = msg.x
            meg.y = msg.y
            meg.theta = 0.0
            self.eraser_spawn_pose_pub.publish(meg)
        self.get_logger().info(f"Mouse Position: x={msg.x}, y={msg.y}")

    def spawn_pizza(self, position):
        position_request = GivePosition.Request()
        position_request.x = position[0]
        position_request.y = position[1]
        self.spawn_pizza_client.call_async(position_request)

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

    def eat_pizza(self):
        eat_request = Empty.Request()
        self.eat_pizza_client.call_async(eat_request)

    def eraser_control_to_target(self):
        if len(self.eraser_target) > 0 and self.eraser_controller_enable and self.eraser_noteatyet == False:
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

            if abs(dx) < 0.1 and abs(dy) < 0.1:
                self.eraser_cmd_vel(0.0, 0.0)
                if not self.eraser_is_eat_all:
                    self.eat_pizza()
                    self.eraser_noteatyet = False
            else:
                self.eraser_cmd_vel(u_dis, u_ori)

    def load_targets_from_yaml(self):
        try:
            with open("targets.yaml", "r") as file:
                loaded_data = yaml.safe_load(file)
                if loaded_data and len(loaded_data) == 4:
                    self.target_queue = loaded_data
                    self.get_logger().info(f"Loaded targets from YAML: {self.target_queue}")
                else:
                    self.get_logger().warning("YAML data format incorrect, using default targets")
        except FileNotFoundError:
            self.get_logger().warning("targets.yaml not found, using default targets")
        except Exception as e:
            self.get_logger().error(f'Failed to load targets.yaml: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()