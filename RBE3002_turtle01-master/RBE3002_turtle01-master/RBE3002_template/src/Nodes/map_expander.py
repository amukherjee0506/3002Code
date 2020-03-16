#!/usr/bin/env python

import rospy
from nav_msgs.srv import GetMap, GetPlan
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
import PriorityQueue
from path_planner import PathPlanner
from copy import deepcopy


class ExpandMap:

    def __init__(self):
        """
        Class constructor
        """
        self.mapdata = OccupancyGrid()
        self.cspace = OccupancyGrid()
        # Initialize the node and call it "map_expander"
        rospy.init_node("map_expander")
        # Create a new service called "expand_map" that accepts no messages
        # and calls self.calc_cspace() when a message is received
        # rospy.Subscriber('/map', OccupancyGrid, self.request_map)
        self.expanded_cells = rospy.Publisher('/map_expander/expanded_cells', GridCells, queue_size=10)
        rospy.Service('get_map', GetMap, self.get_this_map)
        # Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Map Expander node ready")

    @staticmethod
    def grid_to_index(mapdata, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """

        return int((y * mapdata.info.width) + x)

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

        # ESTABLISH REQUIRED CONSTANTS
        index = ExpandMap.grid_to_index(mapdata, x, y)
        x_max = mapdata.info.width
        y_max = mapdata.info.height

        walkable = True

        if (0 <= x < x_max) or (0 <= y < y_max):
            if mapdata.data[index] == 0:
                walkable = True
            elif mapdata.data[index] == -1:
                walkable = True
            else:
                walkable = False
        else:
            walkable = False

        return walkable

    @staticmethod
    def neighbors_of_8(mapdata, x, y):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """self.pth = 0

        # CREATES AN EMPTY OUTPUT LIST
        walkable_neighbors = []
        # LISTS ALL NEIGHBORS
        xa = int(x - 1)
        xb = int(x + 1)
        ya = int(y - 1)
        yb = int(y + 1)
        neighbors = [(x, yb), (xa, yb), (xa, y), (xa, ya), (x, ya), (xb, ya), (xb, y), (xb, yb)]
        # ITERATES THROUGH LIST OF NEIGHBORS
        for coord in neighbors:
            # ADDS WALKABLE NEIGHBORS TO THE OUTPUT LIST
            if ExpandMap.is_cell_walkable(mapdata, coord[0], coord[1]):
                walkable_neighbors.append(coord)
        # RETURNS OUTPUT LIST
        return walkable_neighbors


    def request_map(self, msg):
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        rospy.loginfo("looking for path")
        rospy.wait_for_service('dynamic_map')
        try:
            map_service = rospy.ServiceProxy('dynamic_map', GetMap)
            return map_service().map
        except rospy.ServiceException, e:
            print "request_map service call unsuccessful: %s" % e
            return OccupancyGrid()

    def calc_cspace(self, mapdata):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        :return        [OccupancyGrid] The C-Space.
        """

        rospy.loginfo("Calculating C-Space")
        # Go through each cell in the occupancy grid
        # Inflate the obstacles where necessary
        padding = 2
        newmap = deepcopy(mapdata)
        # for each cell in mapdata
        #   if (cell is walkable) and (cell has an obstacle within padding)
        #     add padding in newmap at current cell

        # input (x,y,pad)
        # sx = min(max(0, x-pad), width)
        # sy = min(max(0, y-pad), height)
        # ex = max(min(width, x+pad), 0)
        # ey = max(min(height, y+pad), 0)
        # for each xx in range(sx,ex)
        #   for each yy in range(sy,ey)
        #     if cell(xx,yx) has obstacle
        #        return True
        #   return False

        new_data = []
        for item in mapdata.data:
            new_data.append(item)

        for h in range(mapdata.info.height - 1):
            for w in range(mapdata.info.width - 1):
                neighbors = ExpandMap.neighbors_of_8(mapdata, w, h)
                if len(neighbors) < 8:
                    new_data[ExpandMap.grid_to_index(mapdata, w, h)] = 100

        #Create a GridCells message and publish it
        expanded_cells = GridCells()
        expanded_cells.header.frame_id = "map"
        expanded_cells.cell_width = mapdata.info.resolution
        expanded_cells.cell_height = mapdata.info.resolution
        expanded_cells.cells = []

        for h in range(mapdata.info.height):
            for w in range(mapdata.info.width):
                index = ExpandMap.grid_to_index(mapdata, w, h)
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
        newmap.data = new_data
        return newmap

    def get_this_map(self):
        map = self.request_map()
        return self.calc_cspace(map)

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """

        rospy.spin()


if __name__ == '__main__':
    ExpandMap().run()
