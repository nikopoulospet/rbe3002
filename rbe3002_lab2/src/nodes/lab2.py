#!/usr/bin/env python2

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class Lab2:

    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ### Initialize node, name it 'lab2'
        rospy.init_node('lab2')
        rospy.Rate(10.0)
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.TwistPublisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        self.OdometrySubscriber = rospy.Subscriber("/odom", Odometry, self.update_odometry) 
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        self.PoseSubscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.go_to)

        ### ROBOT PARAMETERS
        self.px = 0 # pose x
        self.py = 0 # pose y
        self.pth = 0 # yaw angle



    def send_speed(self, linear_speed, angular_speed):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        ### REQUIRED CREDIT
        ### Make a new Twist message
        # TODO
        ### Publish the message
        # TODO
        pass # delete this when you implement your code

    
        
    def drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        ### REQUIRED CREDIT
        pass # delete this when you implement your code



    def rotate(self, angle, aspeed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        ### REQUIRED CREDIT
        pass # delete this when you implement your code



    def go_to(self, msg):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        ### REQUIRED CREDIT
        pass # delete this when you implement your code



    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        ### REQUIRED CREDIT
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation # (x,y,z,w)
        quat_list = [quat_orig.x,quat_orig.y,quat_orig.z,quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.pth = yaw
        


    def arc_to(self, position):
        """
        Drives to a given position in an arc.
        :param msg [PoseStamped] The target pose.
        """
        ### EXTRA CREDIT
        # TODO
        pass # delete this when you implement your code



    def smooth_drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        ### EXTRA CREDIT
        # TODO
        pass # delete this when you implement your code



    def run(self):
        rospy.spin()

if __name__ == '__main__':
    Lab2().run()
