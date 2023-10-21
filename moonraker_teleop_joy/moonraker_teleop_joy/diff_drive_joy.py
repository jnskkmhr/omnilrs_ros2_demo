import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
import math

# ROVER dimension (in m)
RADIUS = 90.0 * 10e-3
BASE = 335.0 * 10e-3 # distance between two wheels

# TODO: tune the optimal value
MAX_SPEED = 0.3 #m/s
MAX_ANGULAR_SPEED = 0.10 #rad/s=10deg/s 
MAX_ROT = 50.0 # after deceleration

# joy button setting
DRIVE_AXIS = 1
ROT_AXIS = 3

class DiffDriveJoy(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.pub = self.create_publisher(Float32MultiArray, 'throttle', 10)
        self.sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.twist = [0.0, 0.0]
    
    def _joy_to_twist(self, joy_msg):
        drive_throttle = joy_msg.axes[DRIVE_AXIS]
        rot_throttle = -1 * joy_msg.axes[ROT_AXIS]
        drive_velocity = drive_throttle * MAX_SPEED
        rot_velocity = rot_throttle * MAX_ANGULAR_SPEED
        self.twist = [drive_velocity, rot_velocity]

    def controller(self):
        """
        Hardcoded inside the method.
        controller method takes twist list (vel_x, ang_z)
        In the future, controller class is implemented separetely
        And can be switched by parameter
        """
        Twist = self.twist
        vel = Twist[0]
        ang = Twist[1]
        rpm_r = (vel + ang * (BASE/2)) / RADIUS
        rpm_l = (vel - ang * (BASE/2)) / RADIUS
        throttle_r = (rpm_r * (60/2*math.pi)) / MAX_ROT
        throttle_l = (rpm_l * (60/2*math.pi)) / MAX_ROT
        throttle_command = Float32MultiArray()
        throttle_command.data = [throttle_l, throttle_r]
        return throttle_command
    
    def joy_callback(self, joy_msg):
        self._joy_to_twist(joy_msg)
        throttle_msg = self.controller()
        self.pub.publish(throttle_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveJoy('diff_drive_joy')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()