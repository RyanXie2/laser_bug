import ckbot.logical as L
import threading
import rclpy
from math import pi
import datetime
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from move import Move

WHEEL_DIA = 0.128 # m
WHEEL_LENGTH = 0.1915 # m


class odometry:
    def __init__(self, dt):
        self.pre_theta = 0
        self.pre_x = 0
        self.pre_y = 0
        self.dt = dt
        
    # Runge Kutta integration
    def step(self, v, w):
        k00 = v * np.cos(self.pre_theta)
        k01 = v * np.sin(self.pre_theta)
        k02 = w
        
        k10 = v * np.cos(self.pre_theta + self.dt/2 * k02)
        k11 = v * np.sin(self.pre_theta + self.dt/2 * k02)
        k12 = w
        
        k20 = v * np.cos(self.pre_theta + self.dt/2 * k12)
        k21 = v * np.sin(self.pre_theta + self.dt/2 * k12)
        k22 = w
        
        k30 = v * np.cos(self.pre_theta + self.dt/2 * k22)
        k31 = v * np.sin(self.pre_theta + self.dt/2 * k22)
        k32 = w
        
        x = self.pre_x + self.dt/6 * (k00 + 2*(k10 + k20) + k30)
        y = self.pre_y + self.dt/6 * (k01 + 2*(k11 + k21) + k31)
        theta = self.pre_theta + self.dt/6 * (k02 + 2*(k12 + k22) + k32)
        
        self.pre_x = x
        self.pre_y = y
        self.pre_theta = theta
        
        return (x, y, theta)
    
class Odom_pub(Node):
    def __init__(self):
        super().__init__('odom_pub')
        self.odom_pub_ = self.create_publisher(Odometry, '/odom', 10)
        self.hz = 20
        timer_period = 1/self.hz  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.c = L.Cluster(count=2) # Insert the number of physical modules here
        
        
        self.c.at.Nx98.set_mode(1)
        self.c.at.Nx06.set_mode(1)
        self.linear_vel = 0
        self.angular_vel = 0
        self.odom_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.move_callback,
            10)
        
        self.pre_L = self.c.at.Nx98.get_pos() + 18000
        self.pre_R = self.c.at.Nx06.get_pos() + 18000
        self.previous_angular_vel_L = 0
        self.previous_angular_vel_R = 0
        self.odom = odometry(1/self.hz)
        
    def move_callback(self, msg):
        self.linear_vel = -msg.linear.x * 2
        self.angular_vel = -msg.angular.z * 2
        Vl = (2*self.linear_vel + self.angular_vel * WHEEL_LENGTH)/(WHEEL_DIA)
        Vr = (2*self.linear_vel - self.angular_vel * WHEEL_LENGTH)/(WHEEL_DIA)
        
        self.c.at.Nx98.set_speed(Vl)
        self.c.at.Nx06.set_speed(-Vr)
        
    def set(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        
    def stop(self):
        self.linear_vel = 0
        self.angular_vel = 0
    
    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
        return [qx, qy, qz, qw]
    
    def timer_callback(self):
        
        
        cur_L = self.c.at.Nx98.get_pos()+18000
        cur_R = self.c.at.Nx06.get_pos() + 18000
        angular_vel_L = (cur_L - self.pre_L)/100 * pi/180 * self.hz
        angular_vel_R = -(cur_R - self.pre_R)/100 * pi/180 * self.hz
        
        
        self.pre_L = cur_L
        self.pre_R = cur_R
        
        # reject outlier
        if (abs(angular_vel_L) > 30 or abs(angular_vel_R) > 30):
            angular_vel_L = self.previous_angular_vel_L
            angular_vel_R = self.previous_angular_vel_R
        else:
        # store previous velocity for outlier rejection
            self.previous_angular_vel_L = angular_vel_L
            self.previous_angular_vel_R = angular_vel_R
        
        linear_vel_L = angular_vel_L * WHEEL_DIA/2
        linear_vel_R = angular_vel_R * WHEEL_DIA/2
        
        
        robot_linear_vel = (linear_vel_R + linear_vel_L)/2
        robot_angular_vel = (linear_vel_R - linear_vel_L)/WHEEL_LENGTH
        
        x, y, theta = self.odom.step(-robot_linear_vel, robot_angular_vel)
        print("x: ", x)
        print("y: ", y)
        print("theta: ", theta)
        
        odom_message = Odometry()
        odom_message.pose.pose.position.x = x
        odom_message.pose.pose.position.y = y
        quat = self.get_quaternion_from_euler(0,0, theta)
        odom_message.pose.pose.orientation.x = quat[0]
        odom_message.pose.pose.orientation.y = quat[1]
        odom_message.pose.pose.orientation.z = quat[2]
        odom_message.pose.pose.orientation.w = quat[3]

        self.odom_pub_.publish(odom_message)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
    
        
def main(args = None):
    rclpy.init(args = args)
    odom_pub = Odom_pub()
    rclpy.spin(odom_pub)
    
    
if __name__ == '__main__':
    main()
    
    
# rclpy.init()
# node = rclpy.create_node('simple_node')
# c = L.Cluster(count=2) # Insert the number of physical modules here
# pre_L = c.at.Nx98.get_pos() + 18000
# pre_R = c.at.Nx06.get_pos() + 18000
# previous_angular_vel_L = 0
# previous_angular_vel_R = 0

# thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
# thread.start()

# hz = 20
# rate = node.create_rate(hz)
# odom = odometry(1/hz)


# try:
#     while rclpy.ok():
#         odom_pub = Odom_pub()
#         cur_L = c.at.Nx98.get_pos()+18000
#         cur_R = c.at.Nx06.get_pos() + 18000
#         angular_vel_L = (cur_L - pre_L)/100 * pi/180 *hz
#         angular_vel_R = -(cur_R - pre_R)/100 * pi/180 *hz
        
        
#         pre_L = cur_L
#         pre_R = cur_R
        
#         # reject outlier
#         if (abs(angular_vel_L) > 30 or abs(angular_vel_R) > 30):
#             angular_vel_L = previous_angular_vel_L
#             angular_vel_R = previous_angular_vel_R
#         else:
#         # store previous velocity for outlier rejection
#             previous_angular_vel_L = angular_vel_L
#             previous_angular_vel_R = angular_vel_R
        
#         linear_vel_L = angular_vel_L * WHEEL_DIA/2
#         linear_vel_R = angular_vel_R * WHEEL_DIA/2
        
        
#         robot_linear_vel = (linear_vel_R + linear_vel_L)/2
#         robot_angular_vel = (linear_vel_R - linear_vel_L)/WHEEL_LENGTH
        
#         x, y, theta = odom.step(robot_linear_vel, robot_angular_vel)
#         print("x: ", x)
#         print("y: ", y)
#         print("theta: ", theta)
#         odom_pub.set(x, y, theta)
#         rate.sleep()
        
# except KeyboardInterrupt:
#     c.at.Nx98.set_speed(0)
#     c.at.Nx06.set_speed(0)

# rclpy.shutdown()
# thread.join()



# current_pos = c.at.Nx98.get_pos()
# c.at.Nx98.set_pos(current_pos+60000)

