#!/usr/bin/env python2

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class Lab2:

    def __init__(self):
        """
        Class constructor
        """
        rospy.sleep(4)
        ### REQUIRED CREDIT
        ### Initialize node, name it 'lab2'
        rospy.init_node('lab2', anonymous=True)
        rospy.Rate(10.0)
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.TwistPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        self.OdometrySubscriber = rospy.Subscriber("/odom", Odometry, self.update_odometry) 
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        self.PoseSubscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.go_to)

        ### ROBOT PARAMETERS
        self.px = 0 # pose x
        self.py = 0 # pose y
        self.yaw = 0 # yaw angle
        self.maxWheelSpeed = 0.22 #physcial limit by turtlebot 3
        self.maxAngularSpeed = self.maxWheelSpeed*2 / 0.178 # L = 0.178 m
        rospy.sleep(1)
        rospy.loginfo("Lab 2 Node Initalized")


    def send_speed(self, linear_speed, angular_speed):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        ### REQUIRED CREDIT
        if abs(linear_speed) > self.maxWheelSpeed:
            linear_speed = self.maxWheelSpeed * (linear_speed/abs(linear_speed))
        if abs(angular_speed) > self.maxAngularSpeed:
            angular_speed = self.maxAngularSpeed * (angular_speed/abs(angular_speed))
        
        ### Make a new Twist message
        vel_msg = Twist()
            #Linear
        vel_msg.linear.x = linear_speed
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
            #Angular
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = angular_speed
        ### Publish the message
        self.TwistPublisher.publish(vel_msg)

    
        
    def drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """

        init_x = self.px
        init_y = self.py
        init_angle = self.yaw
        tolerance = 0.005
        sleep_time = 0.0250
        # start the robot driving
        self.send_speed(linear_speed, 0)
        # wait till the robot is at the correct pos within the given tolerance
        while abs(distance - math.sqrt((self.px - init_x) ** 2 + (self.py - init_y) ** 2)) > tolerance:
            rospy.sleep(sleep_time)
        # stop the robot from driving
        self.send_speed(0, 0)
        rospy.loginfo("Done Driving")




    def rotate(self, angle, aspeed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        # peramiters
        tolerance = 0.01
        sleep_time = 0.0250
        start_angle = self.yaw
        # start the robot spinning
        d = -1
        if angle < 0: d = 1
        self.send_speed(0,aspeed*d)
        # wait till the robot is at the correct angle within the given tolerance
        while((abs(start_angle - (self.yaw + angle))) > tolerance):
            rospy.sleep(sleep_time)
        # stop the robot from spinning
        self.send_speed(0,0)
        rospy.loginfo("Done Rotating")


    def go_to(self, msg):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y
        quat_orig = msg.pose.orientation # (x,y,z,w)
        quat_list = [quat_orig.x,quat_orig.y,quat_orig.z,quat_orig.w]
        (roll, pitch, target_yaw) = euler_from_quaternion(quat_list)

        to_target_angle = self.yaw - math.atan2(target_y-self.py,target_x-self.px)
        rospy.loginfo(str(to_target_angle))
        self.rotate(to_target_angle, 0.2)

        to_target_distance = math.sqrt((target_x-self.px)**2 + (target_y-self.py)**2)
        self.drive(to_target_distance, 0.1)
        
        to_target_end_angle = self.yaw - target_yaw
        self.rotate(to_target_end_angle, 0.2)



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
        self.yaw = yaw
        


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
        init_x = self.px
        init_y = self.py

        tolerance = 0.005
        sleep_time = 0.0250

        # equation = -4 * (t - 0.5)**2 + 1
        # wait till the robot is at the correct pos within the given tolerance
        while abs(distance - math.sqrt((self.px - init_x) ** 2 + (self.py - init_y) ** 2)) > tolerance:
            t = abs(distance - math.sqrt((self.px - init_x) ** 2 + (self.py - init_y) ** 2))
            self.send_speed(linear_speed * (-3.9 * (t - 0.5)**2 + 1), 0)
            rospy.sleep(sleep_time)
        # stop the robot from driving
        self.send_speed(0, 0)
        rospy.loginfo("Done Smooth Driving")



    def run(self):
        rospy.spin()

if __name__ == '__main__':
    robot = Lab2()
    rospy.sleep(1)
    robot.smooth_drive(1,0.2)
    rospy.spin()