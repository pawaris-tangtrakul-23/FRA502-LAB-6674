#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys, select, termios, tty

# Settings
msg = """
---------------------------
Reading from the keyboard!
---------------------------
Moving around:
   w
 a s d    (Linear X/Y)

   r
   f      (Linear Z)

Frame Control:
   g      : Toggle Reference Frame (World <-> End Effector)

q/z : increase/decrease max speeds by 10%

CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0, 0),
    's': (-1, 0, 0),
    'a': (0, 1, 0),
    'd': (0, -1, 0),
    'r': (0, 0, 1),
    'f': (0, 0, -1),
}

speedBindings = {
    'q': 1.1,
    'z': 0.9,
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_jog_keyboard')
        
        # Publishers
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_frame = self.create_publisher(String, '/teleop_frame', 10)
        
        # State
        self.speed = 0.1
        self.frame = "world" # Default frame
        self.status = 0
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.print_status()
        self.timer = self.create_timer(0.1, self.loop)

    def print_status(self):
        print(msg)
        print(f"Current Speed: {self.speed}")
        print(f"Current Frame: {self.frame.upper()}")

    def loop(self):
        key = getKey(self.settings)
        
        x = 0.0
        y = 0.0
        z = 0.0
        
        if key in moveBindings.keys():
            x = moveBindings[key][0]
            y = moveBindings[key][1]
            z = moveBindings[key][2]
        
        elif key in speedBindings.keys():
            self.speed = self.speed * speedBindings[key]
            print(f"Speed: {self.speed}")

        elif key == 'g':
            # Toggle Frame
            if self.frame == "world":
                self.frame = "end_effector"
            else:
                self.frame = "world"
            
            # Publish new frame setting
            frame_msg = String()
            frame_msg.data = self.frame
            self.pub_frame.publish(frame_msg)
            print(f"Switched to: {self.frame.upper()} FRAME")

        elif key == '\x03': # Ctrl-C
            raise KeyboardInterrupt

        # Publish Twist
        twist = Twist()
        twist.linear.x = x * self.speed
        twist.linear.y = y * self.speed
        twist.linear.z = z * self.speed
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.pub_vel.publish(twist)

def main():
    rclpy.init()
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
