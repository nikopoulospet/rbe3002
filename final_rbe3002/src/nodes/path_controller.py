#!/usr/bin/env python

import math
import rospy
import numpy as np
import cv2
from path_planner import PathPlanner
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from nav_msgs.srv import GetMap, GetMapResponse, GetPlan, GetPlanResponse
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

class PathController:

    def __init__(self):
        # Initialize the node and call it "Path Controller"
        rospy.init_node("path_controller", anonymous=True)
        rospy.Rate(10.0)
        ### Tell ROS that this node publishes a Path message on the "/current_path" topic
        self.PathPublisher = rospy.Publisher("/current_path", Path, queue_size=1)
        self.MapPublisher = rospy.Publisher("/map", OccupancyGrid, queue_size=1)
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.handle_nav_goal
        self.PoseSubscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.handle_nav_goal)
        
        self.get_pathService = rospy.Service("next_path", GetPlan, self.get_path)

        self.prev_state = 0
        self.new_nav_goal = False
        rospy.sleep(3.0)
        rospy.loginfo("Path Controller Node Initalized")

    def handle_nav_goal(self, msg):
        """
        Updates the current 2D Nav Goal
        :param msg [PoseStamped] the 2D Nav Goal
        """
        self.nav_target = msg
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


    def get_path(self, msg):
        """
        """

        phase = 1
        target = None
        plan = None
        while not target or not plan:
            if phase == 1:
                target = PoseStamped()

            elif phase == 2:
                target = msg.goal

            elif phase == 3:
                target = self.wait2DNavGoal()
            
            plan = self.navToPoint(msg.start, target, phase)

        return GetPlanResponse(plan.plan)

    def navToPoint(self,cur_pose, goal, phase):
        rospy.loginfo("Navigating to next point | In Phase #" + str(phase))
        rospy.wait_for_service("plan_path")
        if not goal:
            return None
        try:
            plan = rospy.ServiceProxy('plan_path', GetPlan)
            response = plan(cur_pose, goal, phase) # use tolerance feild for phase info
            if len(response.plan.poses) < 1:
                return None
            return response
        except rospy.ServiceException as e:
            rospy.loginfo("Service failed: %s"%e)


    def wait2DNavGoal(self):
        while not self.new_nav_goal:
            rospy.sleep(0.5)
        return self.nav_target

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    PC = PathController()
    PC.run()
    

        