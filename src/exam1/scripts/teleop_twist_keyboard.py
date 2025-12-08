#!/usr/bin/env python3

import sys
import threading

from example_interfaces.srv import AddTwoInts
import geometry_msgs.msg
import rclpy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
WASD Turtle Control
---------------------------
Moving around:
   W : forward
   A : turn left  
   S : backward
   D : turn right

Speed controls:
q/z : increase/decrease max speeds by 10%

Commands:
p : spawn pizza
e : eat pizza 
r : save pizza position

CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0, 0, 0),    
    'a': (0, 0, 0, 1),    
    's': (-1, 0, 0, 0),   
    'd': (0, 0, 0, -1),   
    'W': (1, 0, 0, 0),    
    'A': (0, 0, 0, 1),    
    'S': (-1, 0, 0, 0),   
    'D': (0, 0, 0, -1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
}

Command = {
    'p' : (1, 0), # for spawn pizza
    'r' : (2, 0), # for save pizza position
    'e' : (3, 0), # for eat pizza

    'P' : (1, 0), 
    'R' : (2, 0),
    'E' : (3, 0),
}

def getKey(settings):
    if sys.platform == 'win32':
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)

def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')

    # parameters
    stamped = node.declare_parameter('stamped', False).value
    frame_id = node.declare_parameter('frame_id', '').value
    
    # --- FIX: Declare parameters for Service and Topic names (Avoid Hard Coding) ---
    # Default to generic names, expecting user to remap via CLI
    service_name = node.declare_parameter('service_name', 'order').value
    cmd_vel_topic = node.declare_parameter('cmd_vel_topic', 'cmd_vel').value

    # Create Client using the parameter/generic name
    Order = node.create_client(AddTwoInts, service_name)
    order_req = AddTwoInts.Request()

    if not stamped and frame_id:
        raise Exception("'frame_id' can only be set when 'stamped' is True")

    if stamped:
        TwistMsg = geometry_msgs.msg.TwistStamped
    else:
        TwistMsg = geometry_msgs.msg.Twist

    # --- FIX: Use the generic topic name (no more /sim1/teleoper/...) ---
    pub = node.create_publisher(TwistMsg, cmd_vel_topic, 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    speed = 0.5
    turn = 1.0
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0

    twist_msg = TwistMsg()

    if stamped:
        twist = twist_msg.twist
        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_msg.header.frame_id = frame_id
    else:
        twist = twist_msg

    try:
        print(msg)
        print(vels(speed, turn))
        # print(f"Publishing to: {cmd_vel_topic}") # Debug info
        # print(f"Service Call to: {service_name}") # Debug info
        
        while True:
            key = getKey(settings)
            
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
                    
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
                
            elif key in Command.keys():
                order_req.a = Command[key][0]
                order_req.b = Command[key][1]
                # Calls the service asynchronously
                Order.call_async(order_req)
                print(f"Command sent: {key} to service '{service_name}'")
                
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                if (key == '\x03'):
                    break

            if stamped:
                twist_msg.header.stamp = node.get_clock().now().to_msg()

            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            pub.publish(twist_msg)

    except Exception as e:
        print(f"Error: {e}")

    finally:
        if stamped:
            twist_msg.header.stamp = node.get_clock().now().to_msg()

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist_msg)
        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)

if __name__ == '__main__':
    main()