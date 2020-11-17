#!/usr/bin/env python

import math
import rospy

class PathController:

    def _init_(self):
        # Initialize the node and call it "Path Controller"
        rospy.init_node("Path Controller")
        rospy.Rate(10.0)

        self.state = {0 : self.findFrontier,
           1 : self.pickFrontier,
           2 : self.navToPoint,
           3 : self.wait2DNavGoal,
           4 : self.goToStart,
        }

        rospy.sleep(1.0)
        rospy.loginfo("Path Controller Node Initalized")

    def findFrontier(self):
        pass

    def pickFrontier(self):
        pass

    def navToPoint(self):
        pass

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
    

        