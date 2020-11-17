#!/usr/bin/env python

class PathController

    def _init_(self):
        # Initialize the node and call it "Path Controller"
        rospy.init_node("Path Controller")
        rospy.Rate(10.0)

        options = {0 : findFrontier,
           1 : pickFrontier,
           2 : navToPoint,
           3 : wait2DNavGoal,
           4 : goToStart,
        }

        rospy.sleep()

    def findFrontier(self)

    def pickFrontier(self)

    def navToPoint(self)

    def wait2DNavGoal(self)

    def goToStart(self)

    def doneNav(self)

    def foundWholeMap(self)

    

        