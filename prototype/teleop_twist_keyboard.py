#!/usr/bin/env python
import rclpy
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import threading
import time

settings = termios.tcgetattr(sys.stdin)

msg = """
Reading from the keyboard and Publishing to Twist at 10Hz!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

class TeleopTwistKeyboard:
    def __init__(self):
        self.speed = 0.25
        self.turn = 1.0
        self.x = 0
        self.y = 0
        self.z = 0
        self.th = 0
        self.status = 0
        self.running = True
        self.lock = threading.Lock()

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        # Non-blocking read with timeout
        if select.select([sys.stdin], [], [], 0.01)[0]:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def vels(self):
        return "currently:\tspeed %s\tturn %s " % (self.speed, self.turn)

    def keyboard_thread(self):
        """Thread function to handle keyboard input"""
        while self.running:
            key = self.getKey()
            
            if key == '':
                continue
                
            with self.lock:
                if key in moveBindings.keys():
                    self.x = moveBindings[key][0]
                    self.y = moveBindings[key][1]
                    self.z = moveBindings[key][2]
                    self.th = moveBindings[key][3]
                elif key in speedBindings.keys():
                    self.speed = self.speed * speedBindings[key][0]
                    self.turn = self.turn * speedBindings[key][1]
                    print(self.vels())
                    if (self.status == 14):
                        print(msg)
                    self.status = (self.status + 1) % 15
                else:
                    # Stop on any other key
                    self.x = 0
                    self.y = 0
                    self.z = 0
                    self.th = 0
                    if (key == '\x03'):  # Ctrl+C
                        self.running = False
                        break

    def publish_thread(self, publisher):
        """Thread function to publish at 10Hz"""
        rate = 10.0  # Hz
        dt = 1.0 / rate
        
        while self.running:
            start_time = time.time()
            
            # Get current state safely
            with self.lock:
                current_x = self.x
                current_y = self.y
                current_z = self.z
                current_th = self.th
                current_speed = self.speed
                current_turn = self.turn
            
            # Create and publish twist message
            twist = Twist()
            twist.linear.x = current_x * current_speed
            twist.linear.y = current_y * current_speed
            twist.linear.z = current_z * current_speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = current_th * current_turn
            
            publisher.publish(twist)
            
            # Maintain 10Hz rate
            elapsed = time.time() - start_time
            sleep_time = max(0, dt - elapsed)
            time.sleep(sleep_time)

def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)
    node = rclpy.create_node('teleop_twist_keyboard')
    
    pub = node.create_publisher(Twist, 'cmd_vel', QoSProfile(depth=10))
    
    teleop = TeleopTwistKeyboard()
    
    try:
        print(msg)
        print(teleop.vels())
        
        # Start keyboard input thread
        keyboard_thread = threading.Thread(target=teleop.keyboard_thread)
        keyboard_thread.daemon = True
        keyboard_thread.start()
        
        # Start publishing thread
        publish_thread = threading.Thread(target=teleop.publish_thread, args=(pub,))
        publish_thread.daemon = True
        publish_thread.start()
        
        # Keep main thread alive and handle ROS2 spinning
        while teleop.running:
            rclpy.spin_once(node, timeout_sec=0.1)
            
    except KeyboardInterrupt:
        teleop.running = False
        
    except Exception as e:
        print(e)
        teleop.running = False

    finally:
        # Send stop command
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        
        # Clean up
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()