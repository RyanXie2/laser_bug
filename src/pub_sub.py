# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self,name=None):
        super().__init__('minimal_publisher')
        
        self.wpoints = 'wpoint not set'
        if name == "rpos":
            self.publisher_ = self.create_publisher(String,'robotposition',10)
        else:
            self.publisher_ = self.create_publisher(String, 'waypoints', 10)
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = '%s' % self.wpoints
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

class MinimalSubscriber(Node):

    def __init__(self, name = None):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
                String,
                'waypoints',
                self.listener_callback,
                10)
        self.subscription  # prevent unused variable warning
        self.wpoints = ""

    def listener_callback(self, msg):
        self.wpoints = msg.data
        
class RobPosSubscriber(Node):

    def __init__(self, name = None):
        super().__init__('robot_pos_subscriber')
        self.subscription = self.create_subscription(Odometry,'odom',self.listener_callback,10)
        self.subscription  # prevent unused variable warning
        self.x = 0
        self.y = 0

    def listener_callback(self, msg):
        self.x = msg.pose.pose.position.x * 100
        self.y = msg.pose.pose.position.y * 100
