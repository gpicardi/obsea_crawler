#!/usr/bin/env python3

from my_bot_interfaces.srv import WaypointFollowing

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty

import numpy as np
from math import sin, cos, pi
import math, time, threading, signal

should_quit = False
def handler(signum, frame):
    global should_quit
    should_quit = True


class MyBotController(Node):

    last_pose = np.array([0, 0, 0])
    current_pose = np.array([0, 0, 0])
    cc = 0 #iteration counter for the control loop
    ctrl_timestep = 0.01 #[s]
    behaviour = None
    behaviour_on = False

    def __init__(self):
        super().__init__('my_bot_controller')

        #create a subscriber to the /odom topic
        self.odometry_subscriber_proxy = self.create_subscription(Odometry, 'odom', self.get_odometry_callback, 10)
        self.odometry_subscriber_proxy  # prevent unused variable warning

        #create a publisher on the /cmd_vel topic which activates with a timer
        self.cmdvel_publisher_proxy = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.set_cmd_vel_callback)

        #create a service to implement waypoint following behaviour
        self.waypoint_following_service_proxy = self.create_service(WaypointFollowing, 'waypoint_following', self.waypoint_following_callback)

        #create a service to stop a behaviour
        self.stop_behaviour_service_proxy = self.create_service(Empty, 'stop_behaviour', self.stop_behaviour_callback)
        print("Robot controller successfully initialized")

    def loop(self):
        print("Robot control loop successfully started")
        """rosthread = threading.Thread(target=self.ros_spin())
        rosthread.deamon = True
        rosthread.start()"""

        while True:
            self.cc+=1
            #print(should_quit)
            if should_quit:
                print("Exiting robot control loop")
                break
            if self.behaviour_on:
                if self.behaviour == "waypoint_following":
                    #apply control law
                    a = 1 #random code line to fix indentation warnings
                else:
                    print("Behavior not selected")
            time.sleep(self.ctrl_timestep)
        if self.behaviour_on:
            self.behaviour_on = False
            self.behaviour = None
        print('end control loop')

    #subscriber saves the current_pose in a member variable to make it accessible for all member functions
    def get_odometry_callback(self, msg):
        self.last_pose = self.current_pose
        roll, pitch, yaw = euler_from_quaternion(msg.pose.pose.orientation) #convert orientation in quaternion to euler
        self.current_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]) #reduced representation of the pose [x,y,yaw]

    #publisher is independent from subscriber. For closed loop control law use self.last_pose and self.current_pose
    def set_cmd_vel_callback(self):
        if self.behaviour_on == True:
            if self.behaviour == 'waypoint_following':
                u = Twist()
                u.linear.x = (self.cc%2)*1.0
                u.angular.z = 0.0

                # publish the control signal on the cmd_vel topic
                self.cmdvel_publisher_proxy.publish(u)
                self.get_logger().info('Publishing: "%s"' % u)
        
    #service to specify the waypoints    
    def waypoint_following_callback(self, request, response):
        #SET MEMBER VARIABLES USED IN THE LOOP
        print(request.waypoints.data[0]+request.waypoints.data[1])
        response.result.data = True
        response.message.data = "Eureka"
        self.get_logger().info('Eureka')
        self.behaviour = 'waypoint_following'
        return response
    
    def stop_behaviour_callback(self, request, response):
        self.behaviour_on = False
        print("stop_behaviour")
        return response
    
    """def ros_spin(my_bot_controller):
       rclpy.spin(my_bot_controller)"""
    
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

    # prevhand is the handler set by the ROS runtime
    prevhand = signal.signal(signal.SIGINT, handler)

    my_bot_controller = MyBotController()
    my_bot_controller.loop()

    #rclpy.spin(my_bot_controller)

    # this is to stop the rospy.spin thread
    prevhand(signal.SIGINT, None)

    rclpy.shutdown()


if __name__ == '__main__':
    main()