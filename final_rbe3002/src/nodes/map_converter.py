#!/usr/bin/env python2

import rospy
from nav_msgs.srv import GetMap, GetMapResponse

class map_converter:
    
    def __init__(self):
        rospy.init_node("map_converter")
        rospy.Rate(10)
        # self.Map_subscriber = rospy.Subscriber("/map")
        self.map_service = rospy.Service("static_map", GetMap, self.serve_map)

    def serve_map(self, foo):
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        rospy.loginfo("Requesting the map from map converter")
        rospy.wait_for_service('dynamic_map')
        try:
            map_service = rospy.ServiceProxy('dynamic_map', GetMap)
            responce = map_service()
            return responce.map
        except rospy.ServiceException as e:
            rospy.loginfo("service call failed: %s" %e)
            return None

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    mc = map_converter()
    mc.run()