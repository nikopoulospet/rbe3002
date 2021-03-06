#!/usr/bin/env python
import rospy
import time
from nav_msgs.msg import Path, OccupancyGrid
from nav_msgs.srv import GetPlan, GetPlanResponse
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

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
        self.MapFound = rospy.Subscriber("/whole_map_found", Bool, self.updatePhase)
        self.get_pathService = rospy.Service("next_path", GetPlan, self.get_path)

        self.phase = 1
        self.initial_pose = None
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

    def wait2DNavGoal(self):
        """
        wait until we have a 2d nav goal
        """
        while not self.new_nav_goal:
            rospy.sleep(1)
        return self.nav_target

    def updatePhase(self, msg):
        """
        update phase given msg from fronteir explorer
        """
        if msg.data:
            self.phase = 2

    def get_path(self, msg):
        """
        upon receving a service call, loop until we have a path from path planner
        """
        if(self.initial_pose == None):
            self.initial_pose = msg.start
        target = None
        plan = None
        
        while not target or not plan:
            phase = self.phase
            if phase == 1:
                target = PoseStamped()

            elif phase == 2: #remove? 
                target = self.initial_pose
            elif phase == 3:
                if(msg.goal == PoseStamped()):
                    target = self.wait2DNavGoal()
                    self.new_nav_goal = False
                else:
                    target = msg.goal

            
            plan = self.navToPoint(msg.start, target, phase)
            if not plan:
                rospy.sleep(1)
        if(self.phase == 2):
            time.sleep(5)
            self.phase = 3
        return GetPlanResponse(plan.plan)

    @staticmethod
    def navToPoint(cur_pose, goal, phase):
        rospy.loginfo("Navigating to next point | In Phase #" + str(phase) + str(goal))
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

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()

if __name__ == "__main__":
    PC = PathController()
    PC.run()
    

        