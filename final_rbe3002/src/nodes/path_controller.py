#!/usr/bin/env python

import math
import rospy
from nav_msgs.msg import Odometry, Path
from nav_msgs.srv import GetMap, GetPlan
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

class PathController:

    def _init_(self):
        # Initialize the node and call it "Path Controller"
        rospy.init_node("Path Controller")
        rospy.Rate(10.0)
        ### Tell ROS that this node publishes a Path message on the "/current_path" topic
        self.PathPublisher = rospy.Publisher("/current_path", Path, queue_size=1)
        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        self.OdometrySubscriber = rospy.Subscriber("/odom", Odometry, self.update_odometry) 
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.handle_nav_goal
        self.PoseSubscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.handle_nav_goal)
        
        self.state = {
        0 : self.findFrontier,
        1 : self.pickFrontier,
        2 : self.navToPoint,
        3 : self.wait2DNavGoal,
        4 : self.goToStart,
        }

        self.robot_x = 0
        self.robot_y = 0
        self.robot_yaw = 0
        self.prev_state = 0

        rospy.sleep(1.0)
        rospy.loginfo("Path Controller Node Initalized")

    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        # read the pos from the message
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        # convert the quarerniaon to a euler angle
        quat_orig = msg.pose.pose.orientation # (x,y,z,w)
        quat_list = [quat_orig.x,quat_orig.y,quat_orig.z,quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        # update the save yaw
        self.robot_yaw = yaw

    def handle_nav_goal(self, msg):
        """
        Updates the current 2D Nav Goal
        :param msg [PoseStamped] the 2D Nav Goal
        """
        self.nav_target = msg.pose
        self.new_nav_goal = True

    @staticmethod
    def request_map():
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        rospy.loginfo("Requesting the map")
        rospy.wait_for_service('dynamic_map')
        try:
            static_map_service = rospy.ServiceProxy('dynamic_map', GetMap)
            responce = static_map_service()
            return responce.map
        except rospy.ServiceException as e:
            rospy.loginfo("service call failed: %s" %e)
            return None

    def findFrontier(self):
        frontierList = []
        mapdata = PathController.request_map()

        for i in range(len(mapdata.data)):
            




        return frontierList

    def pickFrontier(self):
        pass

    def navToPoint(self, goal):
        rospy.loginfo("Navigating to next point")
        rospy.wait_for_service("plan_path")

        cur_pose = PoseStamped()
        cur_pose.pose.position.x = self.robot_x
        cur_pose.pose.position.y = self.robot_y
        try:
            plan = rospy.ServiceProxy('plan_path', GetPlan)
            response = plan(cur_pose, goal, 0.1)
            self.PathPublisher.publish(response)
        except rospy.ServiceException as e:
            rospy.loginfo("Service failed: %s"%e)
        
        while not self.doneNav(goal):
            rospy.sleep(0.5)
        
        if self.prev_state == 1:
            self.state[0]()
        else:
            self.state[3]()


    def wait2DNavGoal(self):
        pass

    def goToStart(self):
        pass

    def doneNav(self):
        pass

    def foundWholeMap(self):
        pass

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    PC = PathController()
    PC.run()
    

