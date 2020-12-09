#!/usr/bin/env python2

import rospy
from nav_msgs.srv import GetMap, GetMapResponse
from std_msgs.msg import Bool

class map_converter:
    
    def __init__(self):
        rospy.init_node("map_converter")
        rospy.Rate(10)
        # self.Map_subscriber = rospy.Subscriber("/map")
        self.map_service = rospy.Service("static_map", GetMap, self.serve_map)
        self.PhaseChanger = rospy.Subscriber("/whole_map_found", Bool, self.change_phase)
        self.phase = 1
        self.static_map = None

    def change_phase(self, msg):
        if(msg.data):
            self.phase = 2
            rospy.loginfo("Requesting the final map")
            rospy.wait_for_service('dynamic_map')
            try:
                map_service = rospy.ServiceProxy('dynamic_map', GetMap)
                responce = map_service()
                self.static_map = responce.map
            except rospy.ServiceException as e:
                rospy.loginfo("service call failed: %s" %e)

    def serve_map(self, foo):
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        if(self.phase == 1):
            rospy.loginfo("Requesting the map for map converter")
            rospy.wait_for_service('dynamic_map')
            try:
                map_service = rospy.ServiceProxy('dynamic_map', GetMap)
                responce = map_service()
                return responce.map
            except rospy.ServiceException as e:
                rospy.loginfo("service call failed: %s" %e)
                return None
        else:
            return self.static_map
                

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    mc = map_converter()
    mc.run()