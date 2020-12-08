#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from path_planner import PathPlanner
from final_rbe3002.msg import keypoint, keypoint_map
from std_msgs.msg import Bool
from nav_msgs.srv import GetMap, GetMapResponse
from nav_msgs.msg import OccupancyGrid, GridCells

class Frontier_Explorer:

    def __init__(self):
        rospy.init_node("Frontier_Explorer", anonymous=True)
        rospy.Rate(10.0)
        ### Tell ROS that this node subsribes to all new paths
        #self.PathPublisher = rospy.Publisher("/new_path", Bool, queue_size=1)  
        ##change state publisher
        self.MapFound = rospy.Publisher("/whole_map_found", Bool, queue_size=1)
        ##publishes cspace to RVIZ
        self.C_spacePublisher = rospy.Publisher("/path_planner/cspace", GridCells, queue_size=1)
        ##publisheds map and keypoints to path planner node to store
        self.Keypoint_map_publisher = rospy.Publisher("/Keypoint_map", keypoint_map, queue_size=1)
        
        rospy.sleep(1.0)
        rospy.loginfo("Frontier Explorer Node Initalized")

    def run(self):
        """
        run the main loop, continue generating and publishing maps to path planner
        once a list of no key points is returned we know we have generated a whole map
        upon a whole map found, send a msg to path controller to switch state
        """
        while 1:
            mapdata = self.get_map()
            Cspacedata = self.calc_cspace(mapdata, 4)
            kp = self.findFrontier(Cspacedata,False)
            if len(kp) == 0:
                break
            kp_simple = self.simplify_keypoints(kp)
            keypoints = self.translate_kp(kp_simple)
            self.send_kp(Cspacedata,keypoints)
        
        msg = Bool()
        msg.data = True
        self.MapFound.publish(msg)
        rospy.sleep(10)
        rospy.signal_shutdown("all fronteirs explored")

    def translate_kp(self, keypoints):
        """
        translate the cv keypoints to a message format
        :param: keypoints [(int,int,float)]
        :return: kp [[keypoint]]
        """
        kp = []
        for kp_ in keypoints:
            temp = keypoint()
            temp.x = kp_[0]
            temp.y = kp_[1]
            temp.size = kp_[2] 
            kp.append(temp)
        return kp

    def send_kp(self,mapdata,keypoints):
        """
        publishes mapdata and keypoints on /Keypoint_map topic
        :param: mapdata [OccupancyGrid]
        :param: keypoints [[keypoint]]
        """
        msg = keypoint_map()
        msg.map = mapdata
        msg.keypoints = keypoints
        self.Keypoint_map_publisher.publish(msg)

    def simplify_keypoints(self, keypoints):
        """
        reduces the CV2 keypoints into tuples for easier processing
        :param: keypoints [cv2.keypoints]
        :return: kp_simple [[(int,int,float)]]
        """
        kp_simple = []
        for kp in keypoints:
            kp_simple.append((int(kp.pt[0]),int(kp.pt[1]),kp.size))
        return kp_simple

    def get_map(self):
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.rospy.rospy.spin().spin()in()
        """
        rospy.wait_for_service('dynamic_map')
        while 1:
            try:
                map_service = rospy.ServiceProxy('dynamic_map', GetMap)
                responce = map_service()
                return responce.map
            except rospy.ServiceException as e:
                rospy.loginfo("service call failed: %s" %e)

    def calc_cspace(self, mapdata, padding):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        # REQUIRED CREDIT
        rospy.loginfo("Calculating C-Space")
        # Go through each cell in the occupancy grid
        # Inflate the obstacles where necessary\

        # define padded grid list
        padded_grid = []
        grid = GridCells()
        grid.header.frame_id = "map"
        grid.cells = padded_grid
        grid.cell_width = mapdata.info.resolution
        grid.cell_height = mapdata.info.resolution
        self.C_spacePublisher.publish(grid)

        padded_map = []
        for cell in mapdata.data:
            padded_map.append(cell)

        # Apply kernel to grid and create padded grid
        x = 0
        y = 0
        test = 0
        for cell_num in range(len(mapdata.data)):
            # cell_num to x,y to check for is occupied
            if mapdata.data[cell_num] > 20: #if a cell is not walkable perform dilation
                test += 1
                for Y in range(-int(padding), 1 + int(padding)):
                    for X in range(-int(padding), 1 + int(padding)):
                        if x+X in range(0, mapdata.info.width) and y+Y in range(0, mapdata.info.width):
                            padded_map[PathPlanner.grid_to_index(mapdata, x+X, y+Y)] = 100 #set grid cell at this index to unwalkable
                            padded_grid.append(PathPlanner.grid_to_world(mapdata, x+X, y+Y))
                            # add all the cells around a blocked cell as long as they are within the grid size
            x += 1
            if x % mapdata.info.width == 0:
                y += 1
                x = 0
        # Create a GridCells message and publish it
        grid = GridCells()
        grid.header.frame_id = "map"
        grid.cells = padded_grid
        grid.cell_width = mapdata.info.resolution
        grid.cell_height = mapdata.info.resolution
        self.C_spacePublisher.publish(grid)
        # Return the C-space with padded map array
        mapdata.data = padded_map
        return mapdata

    def findFrontier(self,mapdata, debug=False):
        """
        generates Keypoints based on map data, this method performes a map service call
        :param: Debug [boolean]
        :return: keypoints [cv2.keypoints]
        """
        rospy.loginfo("calcuating frontiers")
        image = np.array(mapdata.data)
        image = np.reshape(image, (-1,mapdata.info.width))
        walls = np.copy(image)
        walls[walls < 100] = 0
        walls[walls >= 100] = 255
        image[image > 0] = 255
        walls = walls.astype(np.uint8)
        image = image.astype(np.uint8)

        
        evidence_grid = cv2.Canny(image,99,100)
        
        kernel = np.ones((3,3), np.uint8)
        img_dilation = cv2.dilate(evidence_grid, kernel, iterations=2)
        subtracted = np.subtract(img_dilation,walls) 
        fronitiers = cv2.erode(subtracted, np.ones((2,2)), iterations=2)
        combined = cv2.dilate(fronitiers, kernel, iterations=5)
        slimmed = cv2.erode(combined,kernel, iterations=6)
        

        params = cv2.SimpleBlobDetector_Params()
        params.minThreshold = 200
        params.filterByColor = False
        params.filterByArea = False
        params.filterByCircularity = False
        params.filterByConvexity = False
        params.filterByInertia = False

        detector = cv2.SimpleBlobDetector_create(params)
        detector.empty()
        keypoints = detector.detect(slimmed)
        rospy.loginfo("found frontiers")
        if debug:
            #print(keypoints)
            #for key in keypoints:
            #    print(key.size)
            im_with_keypoints = cv2.drawKeypoints(evidence_grid, keypoints, np.array([]), (0,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            cv2.imshow("evidnce_grid",im_with_keypoints)
            cv2.imshow("walls",slimmed)
            cv2.imshow("eg",fronitiers)
            cv2.waitKey(5000)
        
        return keypoints
    
if __name__ == "__main__":
    FE = Frontier_Explorer()
    FE.run()
    

        