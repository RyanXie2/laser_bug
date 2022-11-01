import ckbot.logical as L
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist


WHEEL_DIA = 0.128 # m
WHEEL_LENGTH = 0.1915 # m

class Move(Node):
    def __init__(self):
        super().__init__('move')
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
        
    def move_callback(self, msg):
        self.linear_vel = -msg.linear.x * 2
        self.angular_vel = -msg.angular.z * 2
        Vl = (2*self.linear_vel + self.angular_vel * WHEEL_LENGTH)/(WHEEL_DIA)
        Vr = (2*self.linear_vel - self.angular_vel * WHEEL_LENGTH)/(WHEEL_DIA)
        print(Vl)
        print(Vr)
        
        self.c.at.Nx98.set_speed(Vl)
        self.c.at.Nx06.set_speed(-Vr)
        
    def stop(self):
        self.linear_vel = 0
        self.angular_vel = 0


def main(args = None):
    rclpy.init(args = args)
    move = Move()
    try:
        rclpy.spin(move)
    except KeyboardInterrupt:
        pass
    
    move.stop()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()




