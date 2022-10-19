import imp
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String
from math import atan2, asin, pow, sqrt
from PID_controller import PIDController


class Robot_nav(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.control_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub_ = self.create_publisher(String, '/goal', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.goal_sub = self.create_subscription(
            String,
            '/goal',
            self.goal_callback,
            10)
        
        self.odom_sub  # prevent unused variable warning
        self.goal_sub  # prevent unused variable warning
        
        self.SteerController = PIDController(0.5, 0.001, 0.1)
        
        timer_period = 0.5  # seconds
        self.current_goal = Point() 
        self.current_goal.x = 0.0
        self.current_goal.y = 0.0
        
        self.x = 0
        self.y = 0
        self.theta = 0
        
        self.control_timer = self.create_timer(timer_period, self.control_timer_callback)
        self.goal_timer = self.create_timer(timer_period, self.goal_timer_callback)
        

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

    def cal_dis_to_goal(self):
        dis_to_goal = sqrt(pow(self.current_goal.x - self.x, 2) + pow(self.current_goal.y - self.y, 2))
        return dis_to_goal
    
    def cal_angle_to_goal(self):
        inc_x = self.current_goal.x -self.x
        inc_y = self.current_goal.y -self.y
        angle_to_goal = atan2(inc_y, inc_x)
        return angle_to_goal
    
    def control_timer_callback(self):
        speed = Twist()
        angle_error = self.cal_angle_to_goal() - self.theta
        print("angle error", angle_error)
        dis_error = self.cal_dis_to_goal()
        speed.angular.z = self.SteerController.step(angle_error, 0.5)
        # if abs(angle_error - self.theta) > 0.1:
        #     speed.linear.x = 0.0
        #     speed.angular.z = 0.3
        if abs(dis_error) > 0.2:
            speed.linear.x = 0.5
            #speed.angular.z = 0.0
        else:
            speed.linear.x = 0.0
            speed.angular.z = 0.0
        self.control_pub_.publish(speed)

    def goal_timer_callback(self):
        goal = String()
        goal.data = "5,5 2,0 1,1 3,5 4,2"
        self.goal_pub_.publish(goal)        

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = self.euler_from_quaternion(rot_q.x, rot_q.y, rot_q.z, rot_q.w)
        # self.get_logger().info('Robot angle: {}'.format(theta))
        # print("subscriber")
    
    def goal_callback(self, msg):
        goals_raw = msg.data.split(" ")
        goals_list = []
        for goal in goals_raw:
            temp_point = Point()
            temp_point.x = float(goal.split(",")[0])
            temp_point.y = float(goal.split(",")[1])
            goals_list.append(temp_point)
        if len(goals_list) != 0:
            self.current_goal = goals_list[1]
        else:
            print("You finished all goals!")
        

def main(args = None):
    rclpy.init(args = args)
    robot_nav = Robot_nav()
    rclpy.spin(robot_nav)
    
    
if __name__ == '__main__':
    main()