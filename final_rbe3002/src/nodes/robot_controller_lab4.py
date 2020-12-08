#!/usr/bin/env python2

import rospy
import math
import time
from nav_msgs.msg import Odometry, Path
from nav_msgs.srv import GetPlan
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray
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
        self.OdometrySubscriber = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.update_odometry)
        self.AMCL_OdometrySubscriber = rospy.Subscriber("/odom", Odometry, self.update_odometry)
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        self.PathSubscriber = rospy.Subscriber("/current_path", Path, self.handle_path)
        self.PoseCloudSubscriber = rospy.Subscriber("/particlecloud", PoseArray, self.handle_pose_prob)
        self.PathSubscriber = rospy.Subscriber("/new_path", Bool, self.update_path)
        self.PhaseChanger = rospy.Subscriber("/whole_map_found", Bool, self.change_phase)

        ### ROBOT PARAMETERS
        self.px = 0 # pose x
        self.py = 0 # pose y
        self.yaw = 0 # yaw angle
        self.maxWheelSpeed = 0.22 #physcial limit by turtlebot 3
        self.maxAngularSpeed = self.maxWheelSpeed*2 / 0.178 # L = 0.178 m
        self.cur_path_id = 0
        self.done_nav_flag = True
        self.inital_pos = PoseStamped()
        self.localized = False
        self.phase = 1
        self.new_path = False

        rospy.loginfo("Robot Controller Node Initalized")

    def change_phase(self, msg):
        if(msg.data):
            self.phase = 2
            self.new_path = True

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
        rospy.loginfo("Started Rotate")
        # peramiters
        #tolerance = 0.01
        tolerance = 0.04
        sleep_time = 0.0250
        # intial robot conition
        start_angle = self.yaw
        target_angle = start_angle - angle + math.pi*4
        current_angle = self.yaw + 4*math.pi
        # wait till the robot is at the correct angle within the given tolerance

        Rp = 3
        Ri = 0.003
        Rd = -0.0000

        angular_effort = 0
        angular_effort_old = 0
        last_time = time.time()
        start_time = time.time()
        time.clock()

        angular_error_queue = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

        while((abs(target_angle - (current_angle))) > tolerance):
            if(self.new_path):
                break
            time.sleep(sleep_time)
            ## Angular PID
            # P control on the the robots yaw
            current_angle = self.yaw + 4*math.pi
            angular_error = (current_angle - target_angle + math.pi) % (math.pi*2) - math.pi
            angular_error_p = (angular_error if angular_error > -math.pi else angular_error + (math.pi*2))
            # I control on the robots yaw
            angular_error_queue.pop(0)
            angular_error_queue.append(angular_error_p)
            angular_error_i = sum(angular_error_queue)
            # D control on the robots yaw
            angular_error_d = (angular_effort - angular_effort_old) / (time.time() - last_time)
            # angular effort
            angular_effort_old = angular_effort
            angular_effort = (angular_error_p * Rp) + (angular_error_i * Ri) + (angular_error_d * Rd)
            
            #print(round(angular_effort, 3),round(angular_error_p * Rp, 3),round(angular_error_i * Ri, 3),round(angular_error_d * Rd, 3))

            self.send_speed(0, -angular_effort)
            last_time = time.time()
            # failsafe
            if (time.time() - start_time > 5):
                rospy.loginfo("Rotating Fail-Safe triggered")
                break
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
        self.rotate(to_target_angle, 0.10)

        # DRIVE
        point = msg.pose.position
        self.drive_to_point(point, 0.10)

        # ROTATE 2
        # calculate the angle between current yaw and the target final yaw
        #to_target_end_angle = self.yaw - target_yaw
        #self.rotate(to_target_end_angle, 0.2)
        #self.send_speed(0,0)
        rospy.loginfo("Done With go_to")

    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        msg_type = str(msg._type)
        if ((self.phase == 1 and msg_type == "nav_msgs/Odometry") or (self.phase != 1 and msg_type == "geometry_msgs/PoseWithCovarianceStamped")):
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
        rospy.loginfo("Driving to Point")
        # drive to point drive peramiters
        tolerance = 0.04
        sleep_time = 0.06
        

        # PID variables
        Kp = 4
        Ki = 0.03
        Kd = 0.0001

        Rp = 3
        Ri = 0.001
        Rd = -0.0000
        p_error_old = 0
        angular_effort=0
        angular_effort_old = 0
        last_time = 0
        error_queue = [0,0,0,0,0,0,0,0,0,0]
        angular_error_queue = [0,0,0,0,0,0,0,0,0,0,0,0,0]
        #start of drive to point control loop
        while abs(math.sqrt((point.x - self.px) ** 2 + (point.y - self.py) ** 2)) > tolerance:
            #timing the loop
            if(self.new_path):
                break
            current_time = rospy.get_rostime().nsecs/1000000
            
            ### PID calculations

            ## Linear PID
            # P error calculation
            p_error = abs(math.sqrt((point.x - self.px) ** 2 + (point.y - self.py) ** 2))
            # I error calculation using a queue
            error_queue.pop(0)
            error_queue.append(p_error)
            i_error = sum(error_queue)
            # D error calculation
            d_error = (p_error - p_error_old) / (current_time - last_time)
            # Linear effort
            linear_effort = linear_speed * ((p_error * Kp) + (i_error * Ki) + (d_error * Kd))

            ## Angular PID
            # P control on the the robots yaw
            target_angle = math.atan2((point.y - self.py), (point.x - self.px)) + 4*math.pi
            current_angle = self.yaw + 4*math.pi
            angular_error = (current_angle - target_angle + math.pi) % (math.pi*2) - math.pi
            angular_error_p = (angular_error if angular_error > -math.pi else angular_error + (math.pi*2))
            # I control on the robots yaw
            angular_error_queue.pop(0)
            angular_error_queue.append(angular_error_p)
            angular_error_i = sum(angular_error_queue)
            # D control on the robots yaw
            angular_error_d = (angular_effort - angular_effort_old) / (current_time - last_time)
            # angular effort
            angular_effort_old = angular_effort
            angular_effort = (angular_error_p * Rp) + (angular_error_i * Ri) + (angular_error_d * Rd)
            
            #print(round(angular_effort, 3),round(angular_error_p * Rp, 3),round(angular_error_i * Ri, 3),round(angular_error_d * Rd, 3))
            # send the speeds the linar speed is mulitplied by the scaler based on t
            self.send_speed(linear_effort, -angular_effort)
            # wait
            last_time = current_time
            rospy.sleep(sleep_time)
            # variables for next loop
            p_error_old = p_error
            
            
        # stop the robot from driving
        self.send_speed(0, 0)
        rospy.loginfo("Done Driving to Point")

    def update_path(self, msg):
        self.new_path = msg.data

    def handle_path(self, msg):
        # get the path from the msg
        path = msg.plan.poses

        for point in path:
            self.go_to(point)
            if(self.new_path):
                rospy.loginfo("Path Interupted")
                break
        self.done_nav_flag = True
        rospy.loginfo("Done with Path")
    
    def handle_pose_prob(self, msg):

        tolerance = .25

        sum_x = 0
        sum_y = 0

        sum_dx = 0
        sum_dy = 0

        #cal the avg error in the array
        if(not self.localized):
            #find the average pos
            for i in msg.poses:
                sum_x += i.position.x
                sum_y += i.position.y

            avg_x = sum_x/len(msg.poses)
            avg_y = sum_y/len(msg.poses)

            for i in msg.poses:
                sum_dx += abs(i.position.x - avg_x)
                sum_dy += abs(i.position.y - avg_y)

            avg_dx = sum_dx/len(msg.poses)
            avg_dy = sum_dy/len(msg.poses)

            distance = math.sqrt(avg_dx ** 2 + avg_dy ** 2)

            if distance < tolerance:
                self.send_speed(0,0)
                self.localized = True

            else:
                print(str(distance))
                self.localize()
        else:
            pass

    def localize(self):
        self.send_speed(0,0.3)

    def run(self):
        rospy.wait_for_service('next_path',timeout=None)
        # save the inital positon
        
        self.inital_pos.pose.position.x = self.px
        self.inital_pos.pose.position.y = self.py

        #start localization
        localization = rospy.ServiceProxy('global_localization', Empty)
        null = localization()

        while(1):
            if(self.done_nav_flag):
                if(self.localized):
                    try:
                        #path service call
                        plan = rospy.ServiceProxy('next_path', GetPlan)
                        goal = PoseStamped()

                        #path params
                        cur_pose = PoseStamped()
                        cur_pose.pose.position.x = self.px
                        cur_pose.pose.position.y = self.py

                        #get a path and response
                        response = plan(cur_pose, goal, 0.15)
                        self.done_nav_flag = False
                        self.new_path = False
                        #print(response)
                        self.handle_path(response)
                    except rospy.ServiceException as e:
                        rospy.loginfo("Service failed: %s"%e)
                else:
                    print("NOT LOCALIZED")
            else:
                print("DONE NAV FALG = flase")
            time.sleep(1)

if __name__ == '__main__':
    robot = Robot_controller()
    robot.run()
    rospy.spin()
