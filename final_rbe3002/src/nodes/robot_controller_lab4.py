#!/usr/bin/env python2

import rospy
import math
from nav_msgs.msg import Odometry, Path
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class Robot_controller:

    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ### Initialize node, name it 'Robot_controller'
        rospy.init_node('Robot_controller', anonymous=True)
        rospy.Rate(10.0)
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.TwistPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        self.OdometrySubscriber = rospy.Subscriber("/odom", Odometry, self.update_odometry) 
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        self.PathSubscriber = rospy.Subscriber("/current_path", Path, self.handle_path)
        ### ROBOT PARAMETERS
        self.px = 0 # pose x
        self.py = 0 # pose y
        self.yaw = 0 # yaw angle
        self.maxWheelSpeed = 0.22 #physcial limit by turtlebot 3
        self.maxAngularSpeed = self.maxWheelSpeed*2 / 0.178 # L = 0.178 m
        self.cur_path_id = 0
        self.done_nav_flag = True
        rospy.loginfo("Rbobot Controller Node Initalized")


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

    def rotate(self, angle, aspeed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        # peramiters
        tolerance = 0.01
        sleep_time = 0.0250
        # intial robot conition
        start_angle = self.yaw
        target_angle = start_angle - angle
        # wait till the robot is at the correct angle within the given tolerance
        while((abs(start_angle - (self.yaw + angle))) > tolerance):
            # p controlle on the angle
            angular_error = target_angle - self.yaw
            angular_effort = angular_error * 5
            # send the speeds the linar speed is mulitplied by the scaler based on t
            self.send_speed(0, angular_effort)
        # stop the robot from spinning
        self.send_speed(0,0)
        rospy.loginfo("Done Rotating")


    def go_to(self, msg):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        # Peramiters
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y
        quat_orig = msg.pose.orientation # (x,y,z,w)
        quat_list = [quat_orig.x,quat_orig.y,quat_orig.z,quat_orig.w]
        (roll, pitch, target_yaw) = euler_from_quaternion(quat_list)

        # ROTATE 1
        # calculate the angle between current yaw and the target position
        to_target_angle = self.yaw - math.atan2(target_y-self.py,target_x-self.px)
        #Rotate the robot
        self.rotate(to_target_angle, 0.2)

        # DRIVE
        point = msg.pose.position
        self.drive_to_point(point, 0.2)

        # ROTATE 2
        # calculate the angle between current yaw and the target final yaw
        to_target_end_angle = self.yaw - target_yaw
        self.rotate(to_target_end_angle, 0.2)
        self.send_speed(0,0)
        rospy.loginfo("Done With go_to")

    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        # read the pos from the message
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        # convert the quarerniaon to a euler angle
        quat_orig = msg.pose.pose.orientation # (x,y,z,w)
        quat_list = [quat_orig.x,quat_orig.y,quat_orig.z,quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        # update the save yaw
        self.yaw = yaw

    def drive_to_point(self, point, linear_speed):
        """
        Drives the robot to a point and smoothly changes the speed.
        :param point        [Point]       The target point
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        # the inital robot condition
        init_x = self.px
        init_y = self.py
        distance = abs(math.sqrt((point.x - self.px) ** 2 + (point.y - self.py) ** 2))
        # drive to point drive peramiters
        tolerance = 0.005
        sleep_time = 0.0250
        
        #start of drive to point control loop
        while abs(math.sqrt((point.x - self.px) ** 2 + (point.y - self.py) ** 2)) > tolerance:
            # calculate the persentage of the desired distance travled (0 -> 1)
            t = abs(math.sqrt((self.px - init_x) ** 2 + (self.py - init_y) ** 2)) / distance
            # P control on the the robots yaw
            target_angle = math.atan2((point.y - self.py), (point.x - self.px))
            angular_error = target_angle - self.yaw
            angular_effort = angular_error * 5
            # send the speeds the linar speed is mulitplied by the scaler based on t
            self.send_speed(linear_speed * (-50 * (t - 0.5)**6 + 1), angular_effort)
            # wait
            rospy.sleep(sleep_time)
        # stop the robot from driving
        self.send_speed(0, 0)
        rospy.loginfo("Done Driving to Point")

    def handle_path(self, msg):
        # making it so if a new path comes in this one is canncelled
        self.cur_path_id += 1
        my_id = self.cur_path_id

        # get the path from the msg
        path = msg.plan.poses

        for point in path:
            self.go_to(point)
            if(my_id != self.cur_path_id):
                rospy.loginfo("Path Interupted")
                break
        self.done_nav_flag = True
        rospy.loginfo("Done with Path")


    def run(self):
        rospy.wait_for_service('next_path',timeout=None)
        rospy.wait_for_message('/odom', Odometry)
        while(1):
            if(self.done_nav_flag):
                try:
                    plan = rospy.ServiceProxy('next_path', GetPlan)
                    goal = PoseStamped()
                    cur_pose = PoseStamped()
                    cur_pose.pose.position.x = self.px
                    cur_pose.pose.position.y = self.py

                    response = plan(cur_pose, goal, 0.1)
                    self.done_nav_flag = False
                    self.handle_path(response)
                except rospy.ServiceException as e:
                    rospy.loginfo("Service failed: %s"%e)
            else:
                rospy.sleep(1)

if __name__ == '__main__':
    robot = Robot_controller()
    robot.run()
    rospy.spin()
