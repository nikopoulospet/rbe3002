#!/usr/bin/env python

import math
import rospy
from priority_queue import PriorityQueue
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from tf.transformations import quaternion_from_euler

class PathPlanner:


    
    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ## Initialize the node and call it "path_planner"
        rospy.init_node("path_planner")
        ## Create a new service called "plan_path" that accepts messages of
        ## type GetPlan and calls self.plan_path() when a message is received
        self.path_planService = rospy.Service("plan_path", GetPlan, self.plan_path)
        ## Create a publisher for the C-space (the enlarged occupancy grid)
        ## The topic is "/path_planner/cspace", the message type is GridCells
        self.C_spacePublisher = rospy.Publisher("/path_planner/cspace", GridCells, queue_size = 1)
        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose a the topic names, the message type is GridCells
        self.A_starPublisher = rospy.Publisher("/astar_path", GridCells, queue_size = 1)
        ## Initialize the request counter
        self.requestCounter = 0
        ## Sleep to allow roscore to do some housekeeping
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
        point = Point(real_world_x, real_world_y, 0.)
        # return the point
        return  point


        
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
        return (int(x_cell_index),int(y_cell_index))


        
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
            point = PathPlanner.grid_to_world(mapdata,pose[0],pose[1])
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
        occupied_thresh = 65
        free_thresh = 19.6
        if x < mapdata.info.width and x >= 0 and y < mapdata.info.height and y >= 0:
            cell = PathPlanner.grid_to_index(mapdata,x,y)
            temp = mapdata.data[cell]
            return mapdata.data[cell] < free_thresh
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
            neighbor_left = PathPlanner.grid_to_index(mapdata, x - 1, y)
            neighbors.append(neighbor_left)

        if PathPlanner.is_cell_walkable(mapdata, x + 1, y):
            neighbor_right = PathPlanner.grid_to_index(mapdata, x + 1, y)
            neighbors.append(neighbor_right)

        if PathPlanner.is_cell_walkable(mapdata, x, y + 1):
            neighbor_top = PathPlanner.grid_to_index(mapdata, x, y + 1)
            neighbors.append(neighbor_top)

        if PathPlanner.is_cell_walkable(mapdata, x, y - 1):
            neighbor_down = PathPlanner.grid_to_index(mapdata, x, y - 1)
            neighbors.append(neighbor_down)
        
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
            neighbor1 = PathPlanner.grid_to_index(mapdata, x - 1, y + 1)
            neighbors.append(neighbor1)

        if PathPlanner.is_cell_walkable(mapdata, x , y + 1):
            neighbor2 = PathPlanner.grid_to_index(mapdata, x, y + 1)
            neighbors.append(neighbor2)

        if PathPlanner.is_cell_walkable(mapdata, x + 1, y + 1):
            neighbor3 = PathPlanner.grid_to_index(mapdata, x + 1, y + 1)
            neighbors.append(neighbor3)

        if PathPlanner.is_cell_walkable(mapdata, x - 1, y):
            neighbor4 = PathPlanner.grid_to_index(mapdata, x - 1, y)
            neighbors.append(neighbor4)

        if PathPlanner.is_cell_walkable(mapdata, x + 1, y):
            neighbor5 = PathPlanner.grid_to_index(mapdata, x + 1, y)
            neighbors.append(neighbor5)

        if PathPlanner.is_cell_walkable(mapdata, x - 1, y - 1):
            neighbor6 = PathPlanner.grid_to_index(mapdata, x - 1, y - 1)
            neighbors.append(neighbor6)

        if PathPlanner.is_cell_walkable(mapdata, x, y - 1):
            neighbor7 = PathPlanner.grid_to_index(mapdata, x, y - 1)
            neighbors.append(neighbor7)

        if PathPlanner.is_cell_walkable(mapdata, x + 1, y - 1):
            neighbor8 = PathPlanner.grid_to_index(mapdata, x + 1, y - 1)
            neighbors.append(neighbor8)
        
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
            static_map_service = rospy.ServiceProxy('static_map', GetMap)
            responce = static_map_service()
            return responce.map
        except:
            print("service call failed: %s"%e)
            return None



    def calc_cspace(self, mapdata, padding):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Calculating C-Space")
        ## Go through each cell in the occupancy grid
        ## Inflate the obstacles where necessary\

        #define padded grid list
        padded_grid = []
        padded_map = []

        ## Apply kernel to grid and create padded grid
        x = 0
        y = 0
        test = 0
        for cell_num in range(len(mapdata.data)):
            #cell_num to x,y to check for is occupied            
            if not PathPlanner.is_cell_walkable(mapdata,x,y):
                test +=1
                for Y in range(-int(padding), 1 + int(padding)):
                    for X in range(-int(padding), 1 + int(padding)): 
                        if x+X in range(0,mapdata.info.width) and y+Y in range(0,mapdata.info.width):
                            padded_map.append(100)
                            padded_grid.append(PathPlanner.grid_to_world(mapdata,x+X,y+Y))
                            #add all the cells around a blocked cell as long as they are within the grid size
            else:
                padded_map.append(0)
            x += 1
            if x % mapdata.info.width == 0: 
                y += 1
                x = 0
        ## Create a GridCells message and publish it
        grid = GridCells()
        grid.header.frame_id = "map"
        grid.cells = padded_grid
        grid.cell_width = mapdata.info.resolution
        grid.cell_height = mapdata.info.resolution
        self.C_spacePublisher.publish(grid)
        ## Return the C-space
        #mapdata.data = padded_map
        return mapdata


    
    def a_star(self, mapdata, start, goal):
        ### REQUIRED CREDIT
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))
        Queue = PriorityQueue()
        Queue.put(start,0)
        came_from = {}
        cost = {}
        came_from[start] = 0
        cost[start] = 0

        while not Queue.empty():
            current = Queue.get()

            if current == goal:
                break
            for next in PathPlanner.neighbors_of_4(mapdata,current[0],current[1]):
                print(next)
                new_cost = cost[current] + 1 #add 1 b/c we will be moving by constant cells ? 
                if not next in cost or new_cost < cost[next]:
                    cost[next] = new_cost
                    priority = new_cost + PathPlanner.euclidean_distance(goal[0],goal[1],next[0],next[1])
                    Queue.put(next,priority)
                    came_from[next] = current
        
        path_list = []
        print(current,came_from[current])

        
        path = GridCells()
        path.header.frame_id = "map"
        


        #return Queue.elements()
    
    @staticmethod
    def optimize_path(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        ### EXTRA CREDIT
        rospy.loginfo("Optimizing path")

        

    def path_to_message(self, mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param mapdata  [OccupancyGrid] The map data.
        :param path     [[(int,int)]] The path on the grid (a list of tuples)
        :return         [Path]        A Path message (the coordinates are expressed in the world)
        """
        # initilize the path variable
        world_path = Path()
        # loop through the path
        for index in path:
            # make the current pose
            stamped_pose = PoseStamped()
            # find the point in the real world
            world_point = self.grid_to_world(mapdata, index(0), index(1))
            # add the real world point to the pose
            stamped_pose.pose.point = world_point
            # add the pose to the path
            world_path.poses.append(stamped_pose)
            # if this isnt the first point 
            if( len(world_path.poses) > 1):
                # get the last world point
                last_world_point = self.grid_to_world(mapdata, lastIndex(0), lastIndex(1))
                # calculate the angel between the last and current point
                angle = math.atan2(last_world_point.y-world_point.y, last_world_point.x-world_point.x)
                # convert the angle to quaternion and set it as the angle for the current node
                stamped_pose.pose.quaternion = quaternion_from_euler(yaw, 0, 0)

            lastIndex = index
        
        rospy.loginfo("Returning a Path message")
        return world_path


        
    def plan_path(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req 
        """
        ## Request the map
        ## In case of error, return an empty path
        mapdata = PathPlanner.request_map()
        if mapdata is None:
            return Path()
        ## Calculate the C-space and publish it
        cspacedata = self.calc_cspace(mapdata, 1)
        ## Execute A*
        start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        goal  = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        path  = self.a_star(cspacedata, start, goal)
        ## Optimize waypoints
        #waypoints = PathPlanner.optimize_path(path)
        ## Return a Path message
        return self.path_to_message(mapdata, path)


    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()


        
if __name__ == '__main__':
    pp = PathPlanner()
    pp.run()