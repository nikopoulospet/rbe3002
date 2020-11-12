#!/usr/bin/env python2

import rospy
import math
from nav_msgs.msg import Odometry
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
        self.PoseSubscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.handle_nav_goal)
        ### ROBOT PARAMETERS
        self.px = 0 # pose x
        self.py = 0 # pose y
        self.yaw = 0 # yaw angle
        self.maxWheelSpeed = 0.22 #physcial limit by turtlebot 3
        self.maxAngularSpeed = self.maxWheelSpeed*2 / 0.178 # L = 0.178 m
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

    def drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        # save the inital conditions of the robot
        init_x = self.px
        init_y = self.py
        init_angle = self.yaw
        # drive() peramiters
        tolerance = 0.01
        sleep_time = 0.0250
        # start the robot driving
        self.send_speed(linear_speed, 0)
       
        # control loop for driving
        run = True
        while run:
            # wait till the robot is at the correct distance away within the given tolerance
            if abs(distance - math.sqrt((self.px - init_x) ** 2 + (self.py - init_y) ** 2)) <= tolerance:
                # end the control loop
                run = False
            else:
                # control the robot as it drives
                rospy.sleep(sleep_time)
                # P control on the robots yaw
                angular_error = init_angle - self.yaw
                angular_effort = angular_error * 1
                # send the speed
                self.send_speed(linear_speed,angular_effort)
                
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
        # intial robot conition
        start_angle = self.yaw
        # check to see which direction the robot should rotate
        d = -1
        if angle < 0: d = 1
        # start the robot spinning
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
        # calculate the distance between the current position and target position
        to_target_distance = math.sqrt((target_x-self.px)**2 + (target_y-self.py)**2)
        # drive the calculated distance
        self.smooth_drive(to_target_distance, 0.2)
        # ROTATE 2
        # calculate the angle between current yaw and the target final yaw
        to_target_end_angle = self.yaw - target_yaw
        self.rotate(to_target_end_angle, 0.2)



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
        


    def arc_to(self, position):
        """
        Drives to a given position in an arc.
        :param msg [PoseStamped] The target pose.
        """
        pass


    def smooth_drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        # the inital robot condition
        init_x = self.px
        init_y = self.py
        init_angle = self.yaw
        # smooth drive peramiters
        tolerance = 0.005
        sleep_time = 0.0250

        # equation = -50 * (t - 0.5)**6 + 1
        
        #start of smooth drive control loop
        run = True
        while run:
            # wait till the robot is at the correct pos within the given tolerance
            if abs(distance - math.sqrt((self.px - init_x) ** 2 + (self.py - init_y) ** 2)) <= tolerance:
                run = False
            else:
                # calculate the persentage of the desired distance travled (0 -> 1)
                t = abs(math.sqrt((self.px - init_x) ** 2 + (self.py - init_y) ** 2)) / distance
                # P control on the the robots yaw
                angular_error = init_angle - self.yaw
                angular_effort = angular_error * 10
                # send the speeds the linar speed is mulitplied by the scaler based on t
                self.send_speed(linear_speed * (-50 * (t - 0.5)**6 + 1), angular_effort)
                # wait
                rospy.sleep(sleep_time)
        # stop the robot from driving
        self.send_speed(0, 0)
        rospy.loginfo("Done Smooth Driving")

    def handle_nav_goal(self, msg):

        cur_pose = PoseStamped()

        cur_pose.pose.position.x = self.px
        cur_pose.pose.position.y = self.py

        rospy.wait_for_service('plan_path')

        try:
            plan = rospy.ServiceProxy('plan_path', GetPlan)
            response = plan(cur_pose, msg, 0.1)
            print(response)
            self.handle_a_star(response, msg)
        except rospy.ServiceException as e:
            rospy.loginfo("Service failed: %s"%e)

    def handle_a_star(self, response, goal_pt):

        path = response.plan.poses

        for point in path:
            if point == path[len(path)-1]:
                #if this is the goal
                self.go_to(goal_pt)
            else:
                self.go_to(point)
            rospy.sleep(0.2)
        rospy.loginfo("Done with A Star path")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    robot = Robot_controller()
    rospy.spin()
