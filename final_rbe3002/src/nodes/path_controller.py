#!/usr/bin/env python
import rospy
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
        while not self.new_nav_goal:
            rospy.sleep(0.5)
        return self.nav_target

    def updatePhase(self, msg):
        self.phase = 3

    def get_path(self, msg):
        """
        """
        target = None
        plan = None
        while not target or not plan:
            phase = self.phase
            if phase == 1:
                target = PoseStamped()

            elif phase == 2: #remove? 
                target = msg.goal

            elif phase == 3:
                target = self.wait2DNavGoal()
            
            plan = self.navToPoint(msg.start, target, phase)
            if not plan:
                rospy.sleep(1)

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
        rospy.spin()

if __name__ == "__main__":
    PC = PathController()
    PC.run()
    

        