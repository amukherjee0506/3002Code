#!/usr/bin/env python

import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import OccupancyGrid, GridCells, nav_msgs, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


class Main:

    def __init__(self):
        """
        Class constructor
        """
        self.phase = 1
        self.px = 0
        self.py = 0
        self.pth = 0
        self.starting_x = 0
        self.starting_y = 0
        # Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        # When a message is received, call self.update_odometry
        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        # Subscribe to 2DNavGoal
        #rospy.sleep(2.0)
        rospy.loginfo("Main node ready")
        #-----------------------------------------------------------------------
        # Initialize node, name it 'path_executor'
        #rospy.init_node("path_executor")
        # Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        # When a message is received, call self.update_odometry
        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        # Create a new service called "execute_path" that accepts messages of
        # type Plan and calls self.execute_planned_path() when a message is received
        #rospy.Service('execute_path', Path, self.execute_planned_path)

        rospy.sleep(5)

    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """

        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.pth = yaw

    @staticmethod
    def request_frontier(self):
        """
        Requests a frontier from the frontier server.
        :return Point The best frontier,
                        None in case of error.
        """

        rospy.loginfo("Requesting the Frontier")
        rospy.wait_for_service('find_frontier')
        try:
            Gettable_plan = GetPlan()
            current_pose = PoseStamped()
            current_pose.header.frame_id = "start"
            current_pose.pose.position.x = self.px
            current_pose.pose.position.y = self.py
            current_pose.pose.position.z = 0
            dud_point = PoseStamped()
            dud_point.header.frame_id = "goal"
            dud_point.pose.position.x = 0
            dud_point.pose.position.y = 0
            dud_point.pose.position.z = 0
            Gettable_plan.start = current_pose
            Gettable_plan.goal = dud_point
            Gettable_plan.tolerance = 0
            frontierserver = rospy.ServiceProxy('find_frontier', GetPlan)
            newfrontier = frontierserver(Gettable_plan.start, Gettable_plan.goal, 0)

            return newfrontier
        except rospy.ServiceException, e:
            print "find_frontier service call unsuccessful: %s" % e


    def go_to2(self, x, y):

        rospy.loginfo("looking for path")
        rospy.wait_for_service('plan_path')

        path = rospy.ServiceProxy('plan_path', GetPlan)

        pointnew = PoseStamped()
        pointnew.pose.pose.position.x = x
        pointnew.pose.pose.position.y = y

        path = path(pointnew)

        newposes = path.poses

        for pose in newposes:
            self.go_to(pose)


    def phase_one(self):
        while self.phase == 1:
            frontier = Main.request_frontier(self)
            if frontier[0] == -1515:
                self.phase = 2
                #rospy,sleep(2)
                break
            else:
                self.go_to2(frontier[0], frontier[1])



    def phase_two(self):
        go_to(starting_x, starting_y)



    def send_speed(self, linear_speed, angular_speed):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """

        # Make a new Twist message
        msg_cmd_vel = Twist()
        # Linear
        msg_cmd_vel.linear.x = linear_speed
        msg_cmd_vel.linear.y = 0.0
        msg_cmd_vel.linear.z = 0.0
        # Angular velocity
        msg_cmd_vel.angular.x = 0.0
        msg_cmd_vel.angular.y = 0.0
        msg_cmd_vel.angular.z = angular_speed
        # Publish the message
        self.cmd_vel.publish(msg_cmd_vel)

    def drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """

        init_x = self.px
        init_y = self.py
        distance_traveled = 0
        threshold = .05

        self.send_speed(linear_speed, 0)

        while (distance - distance_traveled) > threshold:
            distance_traveled = math.sqrt((self.px - init_x)**2 + (self.py - init_y)**2)
            rospy.sleep(.050)

        self.send_speed(0, 0)

    def rotate(self, angle, aspeed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param aspeed [float] [rad/s] The angular speed.
        """
        current_angle = self.pth
        goal_angle = current_angle + angle
        threshold = .01

        if angle < 0:
            aspeed = -aspeed

        self.send_speed(0, aspeed)

        while abs(goal_angle - current_angle) > threshold:
            current_angle = self.pth
            rospy.sleep(.050)

        self.send_speed(0, 0)

    def execute_planned_path(self, msg):

        path_now = msg.plan.poses.reverse()
        for requestedPose in path_now.plan.poses:
            self.go_to(requestedPose)

    def go_to(self, msg):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        current_x = self.px
        current_y = self.py
        current_angle = self.pth

        delta_x = msg.pose.position.x - current_x
        delta_y = msg.pose.position.y - current_y
        delta_theta = math.atan2(delta_y, delta_x) - current_angle

        self.rotate(delta_theta, 0.25)
        self.drive(math.sqrt(delta_x * delta_x + delta_y * delta_y), 0.1)

    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """

        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.pth = yaw


    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.sleep(2.0)
        self.starting_x = self.px
        self.starting_y = self.py
        self.phase_one()
        self.phase_two()
        rospy.spin()


if __name__ == '__main__':
    Main().run()
