#!/usr/bin/env python

import math
import rospy
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
        # Create a publisher for the C-space (the enlarged occupancy grid)
        # The topic is "/path_planner/cspace", the message type is GridCells
        self.C_spacePublisher = rospy.Publisher("/path_planner/cspace", GridCells, queue_size=1)
        # Create a publisher for the A-star checked spots (the enlarged occupancy grid)
        # The topic is "/path_planner/astar_checked", the message type is GridCells
        self.A_star_checkedPublisher = rospy.Publisher("/path_planner/astar_checked", GridCells, queue_size=1)
        # Create publishers for A* (expanded cells, frontier, ...)
        # Choose a the topic names, the message type is GridCells
        self.A_starPublisher = rospy.Publisher("/path_planner/astar_path", Path, queue_size=1)
        # Initialize the request counter
        self.requestCounter = 0
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
        point = Point(real_world_x, real_world_y, 0.)
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
        occupied_thresh = 65
        free_thresh = 19.6
        if x < mapdata.info.width and x >= 0 and y < mapdata.info.height and y >= 0:
            cell = PathPlanner.grid_to_index(mapdata, x, y)
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
        rospy.wait_for_service('dynamic_map')
        try:
            static_map_service = rospy.ServiceProxy('dynamic_map', GetMap)
            responce = static_map_service()
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
        sizeOF = (mapdata.info.width * mapdata.info.height)
        padded_map = [0] * sizeOF #set all values in new list to walkable

        # Apply kernel to grid and create padded grid
        x = 0
        y = 0
        test = 0
        for cell_num in range(len(mapdata.data)):
            # cell_num to x,y to check for is occupied
            if not PathPlanner.is_cell_walkable(mapdata, x, y): #if a cell is not walkable perform dilation
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
        # REQUIRED CREDIT
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" %
                      (start[0], start[1], goal[0], goal[1]))
        Queue = PriorityQueue()
        Queue.put(start, 0)
        came_from = {}
        cost = {}
        came_from[start] = 0
        cost[start] = 0
        #initilize check_grid with the starting node
        checked_grid = [PathPlanner.grid_to_world(mapdata, start[0], start[1])]
        
        while not Queue.empty():
            current = Queue.get()
            
            if current == goal:
                #end when we are at the goal
                break
            for next in PathPlanner.neighbors_of_8(mapdata, current[0], current[1]):
                #add 1 b/c we will be moving by constant cells
                new_cost = cost[current] + PathPlanner.euclidean_distance(current[0], current[1], next[0], next[1]) + PathPlanner.calcTurnCost(current, next)
                if not next in cost or new_cost < cost[next]:
                    cost[next] = new_cost
                    priority = new_cost + PathPlanner.euclidean_distance(goal[0], goal[1], next[0], next[1])
                    Queue.put(next, priority)
                    # add this cell to the list of cells visited for rviz vizulization
                    checked_grid.append(PathPlanner.grid_to_world(mapdata, next[0], next[1]))
                    came_from[next] = current
        path_list = []
        #backtrack through came_from dict until at start pos, then reverse list
        while not came_from[current] == 0:
            path_list.append(current)
            current = came_from[current]
        path_list.reverse()

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
        print("Done with A*")
        return path_list

   @staticmethod
    #adds a cost of 0.01 if the robot has to make a turn between the two positions
    def calcTurnCost(self, pos1, pos2):

        deltaX = pos2[0] - pos1[0]
        deltaY = pos2[1] - pos1[1]

        if deltaX == 0 or deltaY == 0:
            turnCost = 0
        else:
            turnCost = 0.01

    return turnCost

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
            print(i, path[i - 1], path[i], path[i + 1])
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
        world_path.header.frame_id = "map"
        # loop through the path
        for index in range(0,len(path)):
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

    
    def plan_path(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req 
        """
        # Request the map
        # In case of error, return an empty path
        mapdata = PathPlanner.request_map()
        if mapdata is None:
            return Path()
        # Calculate the C-space and publish it
        cspacedata = self.calc_cspace(mapdata, 2)
        # Execute A*
        start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        goal = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        path = self.a_star(cspacedata, start, goal)
        # Optimize waypoints
        waypoints = PathPlanner.optimize_path(path)
        # Return a Path message
        return GetPlanResponse(self.path_to_message(mapdata, waypoints))

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()


if __name__ == '__main__':
    pp = PathPlanner()
    pp.run()
