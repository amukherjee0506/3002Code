#!/usr/bin/env python2


import path_planner
from nav_msgs.srv import GetPlan, GetMap
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion




# Defines the class
class Lab2:

    def __init__(self):
        """
        Class constructor
        """
        self.px = 0
        self.py = 0
        self.pth = 0

        ### REQUIRED CREDIT
        ### Initialize node, name it 'lab2'
        rospy.init_node("lab2", anonymous = True)
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.cmd_vel = rospy.Publisher('/cmd_vel',Twist, queue_size = 10)
        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        rospy.Subscriber('/odom', Odometry , self.update_odometry)
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.run_this_thing
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.run_this_thing)

        rospy.sleep(5)



    def send_speed(self, linear_speed, angular_speed):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """

        ### REQUIRED CREDIT
        ### Make a new Twist message
        msg_cmd_vel = Twist()
	    #Linear
        msg_cmd_vel.linear.x = linear_speed
        msg_cmd_vel.linear.y = 0.0
        msg_cmd_vel.linear.z = 0.0
	    #Angular velocity
        msg_cmd_vel.angular.x = 0.0
        msg_cmd_vel.angular.y = 0.0
        msg_cmd_vel.angular.z = angular_speed
        ### Publish the message
        self.cmd_vel.publish(msg_cmd_vel)



    def drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        ### REQUIRED CREDIT
        init_x = self.px
        init_y = self.py
        distance_traveled = 0
        threshold = .05

        self.send_speed(linear_speed, 0)

        while ((distance - distance_traveled) > threshold):
            distance_traveled = math.sqrt((self.px - init_x)**2 + (self.py - init_y)**2)
            rospy.sleep(.050)

        self.send_speed(0, 0)



    def rotate(self, angle, aspeed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        current_angle = self.pth
        goal_angle = current_angle + angle
        threshold = .01

        if angle < 0:
            aspeed = -aspeed

        self.send_speed(0, aspeed)

        while (abs(goal_angle - current_angle) > threshold):
            current_angle = self.pth
            rospy.sleep(.050)

        self.send_speed(0, 0)



    def run_this_thing(self, msg):
        plan = GetPlan()
        pose_start = PoseStamped()

        pose_start.header.frame_id = "start"

        pose_start.pose.position.x = self.px
        pose_start.pose.position.y = self.py
        pose_start.pose.position.z = 0

        plan.start = pose_start
        plan.goal = msg
        plan.tolerance = 0.1
        rospy.wait_for_service('plan_path')
        plan_path = rospy.ServiceProxy('plan_path', GetPlan)
        pathnow = plan_path(pose_start, msg, 0.1)

        pathnow.plan.poses.reverse()
        for requestedPose in pathnow.plan.poses:
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

        quat_orig = msg.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)

        self.rotate(delta_theta, 0.25)
        self.drive(math.sqrt(delta_x * delta_x + delta_y * delta_y), 0.1)



    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        ### REQUIRED CREDIT
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.pth = yaw



    def run(self):
        rospy.spin()


if __name__ == '__main__':
    Lab2().run()
