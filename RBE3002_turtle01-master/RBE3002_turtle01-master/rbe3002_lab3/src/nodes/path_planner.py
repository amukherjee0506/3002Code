#!/usr/bin/env python

import math
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
import priority_queue
import lab21


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
        rospy.Service('plan_path', GetPlan, self.plan_path)
        ## Create a publisher for the C-space (the enlarged occupancy grid)
        ## The topic is "/path_planner/cspace", the message type is GridCells
        self.cspace = rospy.Publisher('/path_planner/cspace', GridCells, queue_size=10)
        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose a the topic names, the message type is GridCells
        self.expanded_cells = rospy.Publisher('/path_planner/expanded_cells', GridCells, queue_size=10)
        self.frontier = rospy.Publisher('/path_planner/frontier', GridCells, queue_size=10)
        self.path = rospy.Publisher('/path_planner/path', GridCells, queue_size=10)



        ## Initialize the request counter
        ## TODO:
        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(2.0)
        rospy.loginfo("Path planner node ready")



    @staticmethod
    def grid_to_index(mapdata, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
        ### REQUIRED CREDIT
        return int((y * mapdata.info.width) + x)



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
        ### REQUIRED CREDIT
        ### CALCULATE AND RETURN STRAIGHT LINE EUCLIDIAN DISTANCE
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)



    @staticmethod
    def grid_to_world(mapdata, x, y):
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The cell X coordinate.
        :param y       [int]           The cell Y coordinate.
        :return        [Point]         The position in the world.
        """
        ### REQUIRED CREDIT
        ### ESTABLISH REQUIRED CONSTANTS
        newpt = Point()

        x_origin = mapdata.info.origin.position.x + mapdata.info.resolution / 2
        y_origin = mapdata.info.origin.position.y + mapdata.info.resolution / 2

        ### TRANSFORM COORDINATES
        newpt.x = float((x * mapdata.info.resolution) + x_origin)
        newpt.y = float((y * mapdata.info.resolution) + y_origin)
        ### RETURN TUPLE
        return newpt



    @staticmethod
    def world_to_grid(mapdata, wp):
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        ### REQUIRED CREDIT
        ### ESTABLISH REQUIRED CONSTANTS
        x_origin = mapdata.info.origin.position.x
        y_origin = mapdata.info.origin.position.y
        ### TRANSFORM COORDINATES
        gx = int((wp.x - x_origin) / mapdata.info.resolution)
        gy = int((wp.y - y_origin) / mapdata.info.resolution)
        ### RETURN TUPLE
        output_tuple = (gx, gy)
        return output_tuple



    @staticmethod
    def path_to_poses(mapdata, path):
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        ### REQUIRED CREDIT

        poseList = []

        for pos in path:
            wpps = PoseStamped()
            wpps.header.frame_id = "start"
            wpps.pose.position = PathPlanner.grid_to_world(mapdata,pos[0],pos[1])

            poseList.append(wpps)

        return poseList



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
        ### REQUIRED CREDIT
        ### ESTABLISH REQUIRED CONSTANTS
        index = PathPlanner.grid_to_index(mapdata, x, y)
        x_max = mapdata.info.width
        y_max = mapdata.info.height

        walkable = True

        if (0 <= x < x_max) or (0 <= y < y_max):
            if mapdata.data[index] == 0:
                walkable = True
            else:
                walkable = False
        else:
            walkable = False

        return walkable




    @staticmethod
    def neighbors_of_4(mapdata, x, y):
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        ### REQUIRED CREDIT
        ### CREATES AN EMPTY OUTPUT LIST
        walkable_neighbors = []
        ### LISTS ALL NEIGHBORS
        xa = int(x-1)
        xb = int(x+1)
        ya = int(y-1)
        yb = int(y+1)
        neighbors = [(x, yb), (xa, y), (x, ya), (xb, y)]
        ### ITERATES THROUGH LIST OF NEIGHBORS
        for coord in neighbors:
            ### ADDS WALKABLE NEIGHBORS TO THE OUTPUT LIST
            if PathPlanner.is_cell_walkable(mapdata, coord[0], coord[1]):
                walkable_neighbors.append(coord)
        ### RETURNS OUTPUT LIST
        return walkable_neighbors



    @staticmethod
    def neighbors_of_8(mapdata, x, y):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        ### REQUIRED CREDIT
        ### CREATES AN EMPTY OUTPUT LIST
        walkable_neighbors = []
        ### LISTS ALL NEIGHBORS
        xa = int(x-1)
        xb = int(x+1)
        ya = int(y-1)
        yb = int(y+1)
        neighbors = [(x, yb), (xa, yb), (xa, y), (xa, ya), (x, ya), (xb, ya), (xb, y), (xb, yb)]
        ### ITERATES THROUGH LIST OF NEIGHBORS
        for coord in neighbors:
            ### ADDS WALKABLE NEIGHBORS TO THE OUTPUT LIST
            if PathPlanner.is_cell_walkable(mapdata, coord[0], coord[1]):
                walkable_neighbors.append(coord)
        ### RETURNS OUTPUT LIST
        return walkable_neighbors



    @staticmethod
    def request_map():
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Requesting the map")
        rospy.wait_for_service('/static_map')
        try:
            mapserver = rospy.ServiceProxy('/static_map', GetMap)
            newmap = mapserver()

            return newmap.map
        except rospy.ServiceException, e:
            print "Map service call unsuccessful: %s"%e




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
        #print("START")
        #print(mapdata)
        ## Go through each cell in the occupancy grid
        ## Inflate the obstacles where necessary
        # new_data = mapdata.data[:]
        new_data = []
        for item in mapdata.data:
            new_data.append(item)

        for h in range(mapdata.info.height - 1):
            for w in range(mapdata.info.width - 1):
                neighbors = PathPlanner.neighbors_of_8(mapdata, w, h)
                if len(neighbors) < 8:
                    new_data[PathPlanner.grid_to_index(mapdata, w, h)] = 100

        ## Create a GridCells message and publish it
        expanded_cells = GridCells()
        expanded_cells.header.frame_id = "map"
        expanded_cells.cell_width = mapdata.info.resolution
        expanded_cells.cell_height = mapdata.info.resolution
        expanded_cells.cells = []

        for h in range(mapdata.info.height):
            for w in range(mapdata.info.width):
                index = PathPlanner.grid_to_index(mapdata, w, h)
                if new_data[index] == 100:
                    msg_Point = Point()
                    world_point = PathPlanner.grid_to_world(mapdata, w, h)
                    msg_Point.x = world_point.x
                    msg_Point.y = world_point.y
                    msg_Point.z = 0
                    expanded_cells.cells.append(msg_Point)

        self.expanded_cells.publish(expanded_cells)
        ## Return the C-space
        #print("END")
        #print(mapdata)
        mapdata.data = new_data
        return mapdata



    def a_star(self, mapdata, start, goal):
        ### REQUIRED CREDIT
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))
        afrontier = priority_queue.PriorityQueue()
        afrontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0
        path = []

        the_frontier = GridCells()
        the_frontier.header.frame_id = "map"
        the_frontier.cell_width = mapdata.info.resolution
        the_frontier.cell_height = mapdata.info.resolution
        the_frontier.cells = []

        while not afrontier.empty():
            current = afrontier.get()
            #rospy.loginfo("at point" + str(current))

            if current == goal:
                rospy.loginfo("found the goal")
                break

            current_x = current[0]
            current_y = current[1]

            for this_next in PathPlanner.neighbors_of_8(mapdata, current_x, current_y):
                new_cost = cost_so_far[current] + PathPlanner.euclidean_distance(current_x, current_y, this_next[0], this_next[1])
                if this_next not in cost_so_far or new_cost < cost_so_far[this_next]:
                    cost_so_far[this_next] = new_cost
                    priority = new_cost + PathPlanner.euclidean_distance(this_next[0], this_next[1], goal[0], goal[1])
                    afrontier.put(this_next, priority)
                    came_from[this_next] = current

                    msg_Point = Point()
                    world_point = PathPlanner.grid_to_world(mapdata, this_next[0], this_next[1])
                    msg_Point.x = world_point.x
                    msg_Point.y = world_point.y
                    msg_Point.z = 0
                    the_frontier.cells.append(msg_Point)

            self.frontier.publish(the_frontier)
            rospy.sleep(0.025)

        the_path = GridCells()
        the_path.header.frame_id = "map"
        the_path.cell_width = mapdata.info.resolution
        the_path.cell_height = mapdata.info.resolution
        the_path.cells = []

        rospy.loginfo("shit's goin down")
        while 1 == 1:
            msg_Point = Point()
            world_point = PathPlanner.grid_to_world(mapdata, current[0], current[1])
            msg_Point.x = world_point.x
            msg_Point.y = world_point.y
            msg_Point.z = 0
            the_path.cells.append(msg_Point)
            path.append(current)
            previous_node = came_from[current]

            if previous_node == None:
                break
            current = previous_node
            rospy.loginfo(current)
        self.path.publish(the_path)
        return path



    @staticmethod
    def optimize_path(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        ### EXTRA CREDIT
        rospy.loginfo("Optimizing path")

        ### create the optimised list and include the first location
        revised_list = [(path[0][0], path[0][1])]

        ### go through each subsequent pose in the list except for the final one
        for index in range(2,len(path)):

            ### establish coordinates of the previous, current, and next poses
            last_x = path[index-2][0]
            last_y = path[index-2][1]
            this_x = path[index-1][0]
            this_y = path[index-1][1]
            next_x = path[index][0]
            next_y = path[index][1]

            ### evaluate angles to compare trajectories with and without the current pose
            spanning_angle = math.atan2((next_y-last_y), (next_x-last_x))
            travel_angle = math.atan2((next_y-this_y), (next_x-this_x))

            ### if the current coordinates do not fall between the previous and
            ### next locations add it to the revised list
            if (abs(spanning_angle - travel_angle) > 0.1):
                revised_list.append(path[index-1])
        
        ### add the final location and return the list
        revised_list.append(path[len(path)-1])
        rospy.loginfo("Path optimized")
        return(revised_list)



    def path_to_message(self, mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Returning a Path message")
        path_boi = Path()
        path_boi.header.frame_id = "path"
        path_boi.poses = PathPlanner.path_to_poses(mapdata, path)
        #print(path_boi)
        return path_boi



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
        waypoints = PathPlanner.optimize_path(path)
        ## Return a Path message
        return self.path_to_message(mapdata, waypoints)



    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """

        #map = PathPlanner.request_map()
        #cspace = self.calc_cspace(map, 1)
        #rospy.loginfo("fuck you")
        #astarmap = self.a_star(map, (3,2), (11,27))
        #rospy.loginfo("these motherfuckers got me fucked up")

        rospy.spin()



if __name__ == '__main__':

    PathPlanner().run()
