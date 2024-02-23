#!/usr/bin/env python3

from my_bot_interfaces.srv import WaypointFollowing

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

import numpy as np
from math import sin, cos, pi
import math


class WaypointFollowingService(Node):

    last_pose = np.array([0, 0, 0])
    current_pose = np.array([0, 0, 0])
    i = 0 #set_cmd_vel_callback iteration counter

    def __init__(self):
        super().__init__('waypoint_following_service')

        #create a subscriber to the /odom topic
        self.subscription = self.create_subscription(Odometry, 'odom', self.get_odometry_callback, 10)
        self.subscription  # prevent unused variable warning

        #create a publisher on the /cmd_vel topic which activates with a timer
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.set_cmd_vel_callback)

        #create a service to implement waypoint following behaviour
        self.srv = self.create_service(WaypointFollowing, 'waypoint_following', self.waypoint_following_callback)

    #subscriber saves the current_pose in a member variable to make it accessible for all member functions
    def get_odometry_callback(self, msg):
        self.last_pose = self.current_pose
        roll, pitch, yaw = euler_from_quaternion(msg.pose.pose.orientation) #convert orientation in quaternion to euler
        self.current_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]) #reduced representation of the pose [x,y,yaw]

    #publisher is independent from subscriber. For closed loop control law use self.last_pose and self.current_pose
    def set_cmd_vel_callback(self):
        u = Twist()
        u.linear.x = (self.i%2)*1.0
        u.angular.z = 0.0

        # publish the control signal on the cmd_vel topic
        self.publisher_.publish(u)
        self.get_logger().info('Publishing: "%s"' % u)
        self.i += 1
        
    #service to specify the waypoints    
    def waypoint_following_callback(self, request, response):
        print(request.waypoints.data[0]+request.waypoints.data[1])
        response.result.data = True
        response.message.data = "Eureka"
        self.get_logger().info('Eureka')

        return response
    
def euler_from_quaternion(q):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """

    x = q.x
    y = q.y
    z = q.z
    w = q.w

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians


def main(args=None):
    rclpy.init(args=args)

    waypoint_following_service = WaypointFollowingService()

    rclpy.spin(waypoint_following_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()