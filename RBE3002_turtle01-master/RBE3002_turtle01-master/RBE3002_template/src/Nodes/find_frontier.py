#!/usr/bin/env python

import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import OccupancyGrid, GridCells, nav_msgs
from geometry_msgs.msg import Point, Pose, PoseStamped


class FindFrontier:

    def __init__(self):
        """
        Class constructor
        """
        # Initialize the node and call it "find_frontier"
        rospy.init_node("find_frontier")

        #rospy.Publisher('this_bish_empty', GridCells)
        #rospy.Subscriber('this_bish_empty', GridCells)

        # Create a new service called "find_frontier" that accepts messages of
        # type GetPlan and calls self.plan_path() when a message is received
        rospy.Service('find_frontier', GetPlan, self.find_frontier)

        # Sleep to allow roscore to do some housekeeping
        rospy.sleep(2.0)
        rospy.loginfo("Frontier Finding node ready")

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
    def euclidean_distance(x1, y1, x2, y2):
        """
        Calculates the Euclidean distance between two points.
        :param x1 [int or float] X coordinate of first point.
        :param y1 [int or float] Y coordinate of first point.
        :param x2 [int or float] X coordinate of second point.
        :param y2 [int or float] Y coordinate of second point.
        :return   [float]        The distance.
        """

        # CALCULATE AND RETURN STRAIGHT LINE EUCLIDIAN DISTANCE
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    @staticmethod
    def request_map():
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """

        rospy.loginfo("Requesting the map")
        rospy.wait_for_service('get_map')
        rospy.loginfo("STEP 1 ...................................")

        try:
            rospy.loginfo("STEP 2 ..............................")
            mapserver = rospy.ServiceProxy('get_map', nav_msgs / GetMap)
            rospy.loginfo("STEP 3 ..............................")
            newmap = mapserver()

            return newmap.map

        except rospy.ServiceException, e:
            print "expand_map service call unsuccessful: %s" % e

    @staticmethod
    def unknown_neighbors_of_8(mapdata, x, y, visited):
        """
        Returns the unknown 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """

        # CREATES AN EMPTY OUTPUT LIST
        unknown_neighbors = []
        # LISTS ALL NEIGHBORS
        xa = int(x - 1)
        xb = int(x + 1)
        ya = int(y - 1)
        yb = int(y + 1)
        neighbors = [(x, yb), (xa, yb), (xa, y), (xa, ya), (x, ya), (xb, ya), (xb, y), (xb, yb)]
        # ITERATES THROUGH LIST OF NEIGHBORS
        for coord in neighbors:
            # ADDS UNKNOWN NEIGHBORS TO THE OUTPUT LIST
            unknown_index = FindFrontier.grid_to_index(mapdata, coord[0], coord[1])
            if mapdata.data[unknown_index] == -1 and unknown_index not in visited:
                unknown_neighbors.append(coord)
        # RETURNS OUTPUT LIST
        return unknown_neighbors

    @staticmethod
    def world_to_grid(mapdata, wp):
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """

        # ESTABLISH REQUIRED CONSTANTS
        x_origin = mapdata.info.origin.position.x
        y_origin = mapdata.info.origin.position.y
        # TRANSFORM COORDINATES
        gx = int((wp.x - x_origin) / mapdata.info.resolution)
        gy = int((wp.y - y_origin) / mapdata.info.resolution)
        # RETURN TUPLE
        output_tuple = (gx, gy)
        return output_tuple

    def find_frontier(self, msg):
        """
        Returns a dictionary of frontiers given by {'key',(x,y,size)} in the occupancy grid.
        :return        {'key',(x,y,size)}   A dictionary of frontiers and their sizes.
        """

        # Get a map from the map_expander node
        mapdata = FindFrontier.request_map()
        # Begins counting the number of centroids
        centroid_number = 0
        # Establishes the dictionary of frontiers and the list of visited points
        # frontiers = {}
        visited_points = []
        frontier = [-1515, -1515]
        priority = 0

        [current_x, current_y] = FindFrontier.world_to_grid(mapdata, msg.start.pose.position)

        # Loop through all cells in the occupancy grid
        for h in range(mapdata.info.height - 1):
            for w in range(mapdata.info.width - 1):
                # Calculates index of the point in the grid
                this_index = FindFrontier.grid_to_index(mapdata, w, h)
                # If the point has not been checked
                if this_index not in visited_points:
                    # Add this point to the list of visited points
                    visited_points.append(this_index)
                    # If this point is unknown
                    if mapdata.data[this_index] == -1:
                        # Increase the total number of centroids by one
                        centroid_number += 1
                        # Set the initial size to one
                        size = 1
                        # Adds to the total x and y for averaging later
                        total_x = w
                        total_y = h
                        # Add any adjacent unknown points to the list of unknowns
                        list_of_unknown_points = FindFrontier.unknown_neighbors_of_8(w, h, mapdata, [])
                        # While there are unknown neighbors
                        while not len(list_of_unknown_points) == 0:
                            # Get a point from the list
                            this = list_of_unknown_points.pop
                            this_x = this[0]
                            this_y = this[1]
                            # Increase the size by one
                            size += 1
                            # Add the current x and y to the totals
                            total_x += this_x
                            total_y += this_y
                            # Add any adjacent unknown points to the list of unknowns
                            list_of_unknown_points.append(FindFrontier.unknown_neighbors_of_8(this_x, this_y, mapdata, visited_points))
                            # Get the index for this point
                            this_index = FindFrontier.grid_to_index(mapdata, this_x, this_y)
                            # Add this point to the visited points list
                            visited_points.append(this_index)
                        # Calculate the centroid using the total xs and ys
                        centroid_x = total_x / size
                        centroid_y = total_y / size

                        centroid_distance = FindFrontier.euclidean_distance(centroid_x, centroid_y, current_x, current_y)
                        centroid_priority = size / centroid_distance
                        if centroid_priority > priority:
                            priority = centroid_priority
                            frontier = (centroid_x, centroid_y)
                        # Add the frontier to the dictionary
                        # frontiers[centroid_number] = (centroid_x, centroid_y, size)
        # Return the dictionary
        best_point = PoseStamped()
        best_point.header.frame_id = "frontier"
        best_point.position = [frontier[0], frontier[1], 0]
        best_point.orientation = [0, 0, 0, 0]
        return best_point

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """

        rospy.spin()


if __name__ == '__main__':
    FindFrontier().run()
