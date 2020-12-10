#!/usr/bin/env python

import math, time
import rospy
import numpy as np
import cv2
from std_msgs.msg import Bool
from final_rbe3002.msg import keypoint, keypoint_map
from priority_queue import PriorityQueue
from nav_msgs.srv import GetPlan, GetPlanResponse, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler

class PathPlanner:

    def __init__(self):
        """
        Class constructor
        """
        # REQUIRED CREDIT
        # Initialize the node and call it "path_planner"
        rospy.init_node("path_planner")
        # Create a new service called "plan_path" that accepts messages of
        # type GetPlan and calls self.plan_path() when a message is received
        self.path_planService = rospy.Service("plan_path", GetPlan, self.plan_path)
        #keypoint service will store map and keypoints from frointer explorer until use
        self.KeypointReceiver = rospy.Subscriber("/Keypoint_map",keypoint_map, self.store_data) #TODO
        # Create a publisher for the C-space (the enlarged occupancy grid)
        # The topic is "/path_planner/cspace", the message type is GridCells
        self.C_spacePublisher = rospy.Publisher("/path_planner/cspace", GridCells, queue_size=1)
        # Create a publisher for the A-star checked spots (the enlarged occupancy grid)
        # The topic is "/path_planner/astar_checked", the message type is GridCells
        self.A_star_checkedPublisher = rospy.Publisher("/path_planner/astar_checked", GridCells, queue_size=1)
        # Create publishers for A* (expanded cells, frontier, ...)
        # Choose a the topic names, the message type is GridCells
        self.A_starPublisher = rospy.Publisher("/path_planner/astar_path", Path, queue_size=1)

        self.PathPublisher = rospy.Publisher("/new_path", Bool, queue_size=1)
        

        #stored map and kp data
        self.stored_mapdata = None
        self.stored_keypoints = None
        self.new_data = False
        self.most_recent_path = []
        # Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Path planner node ready")

    @staticmethod
    def grid_to_index(mapdata, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
        # get the width fro mthe map data
        width = mapdata.info.width
        # calculate the index from the given x y coordantes and the width
        index = y * width + x
        # return the index
        return index

    @staticmethod
    def euclidean_distance(x1, y1, x2, y2):
        """
        Calculates the Euclidean distance between two points.
        :param x1 [int or float] X coordinate of first point.
        :param y1 [int or float] Y coordinate of first point.
        :param x2 [int or float] X coordinate of second point.
        :param y2 [int or float] Y coordinate of second point.
        :return   [float]        The distance.
        """
        # find the change in x and the change in y
        dx = x2 - x1
        dy = y2 - y1
        # find the distance using trig
        distance = math.sqrt(dx ** 2 + dy ** 2)
        # return the distance
        return distance

    @staticmethod
    def grid_to_world(mapdata, x, y):
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The cell X coordinate.
        :param y       [int]           The cell Y coordinate.
        :return        [Point]         The position in the world.
        """
        # get the orgion in the real world
        orginX = mapdata.info.origin.position.x
        orginY = mapdata.info.origin.position.y
        # get the size of each cell
        resolution = mapdata.info.resolution
        # get the real world point
        real_world_x = resolution * (x + 0.5) + orginX
        real_world_y = resolution * (y + 0.5) + orginY
        # make the real world Point
        point = Point(real_world_x, real_world_y, 0)
        # return the point
        return point

    @staticmethod
    def world_to_grid(mapdata, wp):
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        # get the world points
        wpX = wp.x
        wpY = wp.y
        # get the orgion in the real world
        orginX = mapdata.info.origin.position.x
        orginY = mapdata.info.origin.position.y
        # get the resolution of each cell
        resolution = mapdata.info.resolution
        # get the wp distance from the maps world orgin
        x_offest_from_cell_orgin = wpX - orginX
        y_offest_from_cell_orgin = wpY - orginY
        # find the distance from the cell's edge
        x_remainder = x_offest_from_cell_orgin % resolution
        y_remainder = y_offest_from_cell_orgin % resolution
        # find the index of the cell
        x_cell_index = (x_offest_from_cell_orgin - x_remainder) / resolution
        y_cell_index = (y_offest_from_cell_orgin - y_remainder) / resolution
        # return the cell
        return (int(x_cell_index), int(y_cell_index))

    @staticmethod
    def path_to_poses(mapdata, path):
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        poses = []
        for pose in path:
            point = PathPlanner.grid_to_world(mapdata, pose[0], pose[1])
            pose_stamped = PoseStamped()
            pose_stamped.pose.position = point
            poses.append(pose_stamped)
        return poses

    @staticmethod
    def is_cell_walkable(mapdata, x, y):
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [boolean]       True if the cell is walkable, False otherwise
        """
        free_thresh = 19.6
        if x < mapdata.info.width and x >= 0 and y < mapdata.info.height and y >= 0:
            cell = PathPlanner.grid_to_index(mapdata, x, y)
            temp = mapdata.data[cell]
            return (mapdata.data[cell] < free_thresh) 
        return False

    @staticmethod
    def neighbors_of_4(mapdata, x, y):
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        neighbors = []
        if PathPlanner.is_cell_walkable(mapdata, x - 1, y):
            neighbors.append((x-1, y))

        if PathPlanner.is_cell_walkable(mapdata, x + 1, y):
            neighbors.append((x+1, y))

        if PathPlanner.is_cell_walkable(mapdata, x, y + 1):
            neighbors.append((x, y+1))

        if PathPlanner.is_cell_walkable(mapdata, x, y - 1):
            neighbors.append((x, y-1))

        return neighbors

    @staticmethod
    def neighbors_of_8(mapdata, x, y):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        neighbors = []

        if PathPlanner.is_cell_walkable(mapdata, x - 1, y + 1):
            neighbors.append((x-1, y+1))

        if PathPlanner.is_cell_walkable(mapdata, x, y + 1):
            neighbors.append((x, y+1))

        if PathPlanner.is_cell_walkable(mapdata, x + 1, y + 1):
            neighbors.append((x+1, y+1))

        if PathPlanner.is_cell_walkable(mapdata, x - 1, y):
            neighbors.append((x-1, y))

        if PathPlanner.is_cell_walkable(mapdata, x + 1, y):
            neighbors.append((x+1, y))

        if PathPlanner.is_cell_walkable(mapdata, x - 1, y - 1):
            neighbors.append((x-1, y-1))

        if PathPlanner.is_cell_walkable(mapdata, x, y - 1):
            neighbors.append((x, y-1))

        if PathPlanner.is_cell_walkable(mapdata, x + 1, y - 1):
            neighbors.append((x+1, y-1))

        return neighbors

    @staticmethod
    def request_map():
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        rospy.loginfo("Requesting the map")
        rospy.wait_for_service('static_map')
        try:
            map_service = rospy.ServiceProxy('static_map', GetMap)
            responce = map_service()
            return responce.map
        except rospy.ServiceException as e:
            rospy.loginfo("service call failed: %s" %e)
            return None

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

        sizeOF = (mapdata.info.width * mapdata.info.height)
        padded_map = []
        for cell in mapdata.data:
            padded_map.append(cell)
            #[0] * sizeOF #set all values in new list to walkable

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
                            padded_map[PathPlanner.grid_to_index(
                                mapdata, x+X, y+Y)] = 100 #set grid cell at this index to unwalkable
                            padded_grid.append(
                                PathPlanner.grid_to_world(mapdata, x+X, y+Y))
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

    def a_star(self, mapdata, start, goal):
        """
        calculates A* for given start and goal pose
        :param: mapdata [OccupancyGrid]
        :param: start [(int,int)]
        :param: goal [(int,int)]
        :return: path_list [[(int,int)]]
        :return: grid [GridCells]
        """
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" %
                      (start[0], start[1], goal[0], goal[1]))
        Queue = PriorityQueue()
        Queue.put(start, 0)
        came_from = {}
        cost = {}
        came_from[start] = 0
        cost[start] = 0
        start_time = time.time()
        time.clock()
        timeout = 3
        #initilize check_grid with the starting node
        checked_grid = [PathPlanner.grid_to_world(mapdata, start[0], start[1])]
        
        while not Queue.empty():
            current = Queue.get()
            if time.time() - start_time > timeout:
                rospy.loginfo("A-Star Fail-Safe triggered")
                return [],GridCells()          
            if current == goal:
                #end when we are at the goal
                break
            for next in PathPlanner.neighbors_of_8(mapdata, current[0], current[1]):
                #add 1 b/c we will be moving by constant cells
                new_cost = cost[current] + PathPlanner.euclidean_distance(current[0], current[1], next[0], next[1]) + PathPlanner.calcTurnCost(came_from[current],current,next)
                if not next in cost or new_cost < cost[next]:
                    cost[next] = new_cost
                    priority = new_cost + PathPlanner.euclidean_distance(goal[0], goal[1], next[0], next[1])
                    Queue.put(next, priority)
                    # add this cell to the list of cells visited for rviz vizulization
                    checked_grid.append(PathPlanner.grid_to_world(mapdata, next[0], next[1]))
                    came_from[next] = current
        path_list = []
        #backtrace through came_from dict until at start pos, then reverse list
        while not came_from[current] == 0:
            path_list.append(current)
            current = came_from[current]
        #path_list.append(current)
        path_list.reverse()
        if len(path_list) > 0:
            path_list.append(goal)

        # Create a GridCells message
        grid = GridCells()
        grid.header.frame_id = "map"
        # add the cells that were visited to the message
        grid.cells = checked_grid
        # add cell info to the message
        grid.cell_width = mapdata.info.resolution
        grid.cell_height = mapdata.info.resolution
        # publish the message
        self.A_star_checkedPublisher.publish(grid)
        return path_list,grid

    @staticmethod
    def calcTurnCost(pos0,pos1, pos2):
        """
        adds a cost of 0.01 if the robot has to make a turn between the two positions
        """
        if pos0 == 0:
            return 0
        theta1 = PathPlanner.calcAngle(pos0,pos1)
        theta2 = PathPlanner.calcAngle(pos1,pos2)    
        return abs(abs(theta1) - abs(theta2)) * 0.025

    @staticmethod
    def calcAngle(start,end):
        """
        calculates the angle between 2 grid points
        :param start [(x,y)]
        :param end [(x,y)]
        :return [float]
        """
        return math.atan2(end[1]-start[1],end[0]-start[0])

    @staticmethod
    def checkJunk(start,end,tolerance):
        """
        Checks to see if the start and end poses are too close to bother generating a path
        :param start [(x,y)]
        :param end [(x,y)]
        :param tolerance [int]
        :return [bool]
        """
        for i in range(start[0]-tolerance,start[0]+tolerance+1):
            for j in range(start[1]-tolerance,start[1]+tolerance+1):
                if end == (i,j):
                    return True
        return False

    @staticmethod
    def optimize_path(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        # EXTRA CREDIT
        rospy.loginfo("Optimizing path")
        # creates a list to store the optimized path
        better_path = [path[0]]
        # creates a variable that stores the slope between the current pt and the pt before
        slope_1 = 0
        # creates a variable that stores the slope between the current pt and the pt after
        slope_2 = 0

        for i in range(1, (len(path) - 1)):
            # check the prevous node
            if (path[i][0] - path[i-1][0]) == 0:
                slope_1 = float('inf')
            else:
                slope_1 = (path[i][1] - path[i-1][1]) / (path[i][0] - path[i-1][0])
            # check the next node
            if (path[i+1][0] - path[i][0]) == 0:
                slope_2 = float('inf')
            else:
                slope_2 = (path[i+1][1] - path[i][1]) / (path[i+1][0] - path[i][0])

            if slope_1 != slope_2:
                better_path.append(path[i])

        better_path.append(path[len(path) - 1])
        return better_path
    
    def path_to_message(self, mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param mapdata  [OccupancyGrid] The map data.
        :param path     [[(int,int)]] The path on the grid (a list of tuples)
        :return         [Path]        A Path message (the coordinates are expressed in the world)
        """
        # initilize the path variable
        world_path = Path()
        world_path.poses = []
        world_path.header.frame_id = "map"
        # loop through the path
        for index in range(len(path)):
            # initilize the current pose
            stamped_pose = PoseStamped()
            # find the point in the real world
            world_point = self.grid_to_world(mapdata, path[index][0], path[index][1])
            # add the real world point to the pose
            stamped_pose.pose.position = world_point

            # Calculate the finale angle for the pose based on the next angle
            if(index == (len(path) - 1)):
                #if the last element do nothing to angle 
                pass
            else:
                # calculate the angel between the last and current point
                angle = math.atan2(path[index+1][1] - path[index][1], path[index+1][0] - path[index][0])
                # convert the angle to quaternion and set it as the angle for the current node
                stamped_pose.pose.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, angle))

            # add the pose to the path
            world_path.poses.append(stamped_pose)

        rospy.loginfo("Returning a Path message")
        # publish the path to rviz
        self.A_starPublisher.publish(world_path)
        # return the path
        return world_path
        
    def pickFrontier(self, mapdata, keypoints, start):
        """
        selects the fronter to navigate to by checking 
        the size of the fronter and size of the path
        :param: mapdata [OccupancyGrid]
        :param: keypoints [[keypoint]]
        :param: start [(int,int)]
        :return: bestPath [[(int,int)]]
        """ 
        max_h = 0
        alpha = 0.99
        beta = 0.01
        bestPath = []
        bestGrid = GridCells()

        for point in keypoints:
            size = point[2]
            point = self.check_if_in_cspace(mapdata, point)
            path,grid = self.a_star(mapdata,start,(point[0],point[1]))
            if not PathPlanner.checkJunk(start,point,5) and len(path) > 0 and max_h < alpha/len(path) + beta * (size/3):
                max_h = alpha/len(path) + beta * (size/3)
                bestPath = path
                bestGrid = grid
        self.A_star_checkedPublisher.publish(bestGrid)
        return bestPath

    def check_if_in_cspace(self,cspacedata,start):
        """
        Checks to see if the robot is in the Cspace, then returns a path to escape
        """
        rospy.loginfo("chaning start pose due to cspace")
        if not PathPlanner.is_cell_walkable(cspacedata,start[0],start[1]):
            count = 0
            while 1:
                if PathPlanner.is_cell_walkable(cspacedata,start[0]+count,start[1]):
                    return (start[0]+count,start[1])
                if PathPlanner.is_cell_walkable(cspacedata,start[0]-count,start[1]):
                    return (start[0]-count,start[1])
                if PathPlanner.is_cell_walkable(cspacedata,start[0],start[1]-count):
                    return (start[0],start[1]-count)
                if PathPlanner.is_cell_walkable(cspacedata,start[0],start[1]+count):
                    return (start[0],start[1]+count)
                count += 1
        else:
            return start

    def check_path_walkable(self,cspacedata,path):
        """
        checks to see if all cells in the path are still walkable
        """
        for cell in path:
            if not PathPlanner.is_cell_walkable(cspacedata,cell[0],cell[1]):
                return False
        return True

    def store_data(self,msg):
        self.new_data = True
        self.stored_mapdata = msg.map
        self.stored_keypoints = self.translate_kp(msg.keypoints)
        if not self.check_path_walkable(msg.map,self.most_recent_path):
            msg = Bool()
            msg.data = True
            self.PathPublisher.publish(msg)
    
    def translate_kp(self, keypoints):
        kp = []
        for kp_ in keypoints:
            kp.append((kp_.x,kp_.y,kp_.size))
        return kp

    def plan_path(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req 
        """
        #path controller indicates phase by msg field
        phase = msg.tolerance
        rospy.loginfo("Planning path")

        if phase == 1:
            # Check for map in storage
            # In case of no map data, return an empty path
            if not self.new_data:
                return Path()
            cspacedata = self.stored_mapdata
            keypoints = self.stored_keypoints
            #extract start pose from msg
            start = self.check_if_in_cspace(cspacedata,PathPlanner.world_to_grid(cspacedata, msg.start.pose.position))
            #calculate a path through heruistic
            path = self.pickFrontier(cspacedata, keypoints,start)
        else:
            # Request the map
            # In case of error, return an empty path
            mapdata = PathPlanner.request_map()
            if mapdata is None:
                return Path()
            cspacedata = self.calc_cspace(mapdata, 3)
            start = self.check_if_in_cspace(cspacedata, PathPlanner.world_to_grid(mapdata, msg.start.pose.position))
            goal = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
            # Calculate the C-space and publish it
            goal = self.check_if_in_cspace(cspacedata, goal)
            # Execute A*
            path, foo = self.a_star(cspacedata, start, goal)

        # Optimize waypoints
        if path == []:
            return Path()
        self.most_recent_path = path
        waypoints = PathPlanner.optimize_path(path)
        # Return a Path message
        return GetPlanResponse(self.path_to_message(cspacedata, waypoints))

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()


if __name__ == '__main__':
    pp = PathPlanner()
    pp.run()
