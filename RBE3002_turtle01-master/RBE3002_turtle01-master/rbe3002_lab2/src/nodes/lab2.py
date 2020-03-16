#!/usr/bin/env python2




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
        #self.odom =
        rospy.Subscriber('/odom', Odometry , self.update_odometry)
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        #self.pose =
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.arc_to)

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
        self.rotate(yaw - math.atan2(delta_y, delta_x), 0.25)



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

        #self.Odometry.publish(msg_cmd_vel)



    def arc_to(self, msg):
        """
        Drives to a given position in an arc.
        :param msg [PoseStamped] The target pose.
        """
        ### EXTRA CREDIT
        current_x = self.px
        current_y = self.py
        current_angle = self.pth
        linear_speed = 0.2

        final_x = msg.pose.position.x
        final_y = msg.pose.position.y
        delta_x = final_x - current_x
        delta_y = final_y - current_y
        delta_theta = math.atan2(delta_y, delta_x) - current_angle + 3.1415/2
        distance = math.sqrt(delta_x * delta_x + delta_y * delta_y)
        rotational_speed = -2 * linear_speed / distance

        self.rotate(delta_theta, 0.25)
        rospy.sleep(0.05)
        target_theta = self.pth + 3.1415
        threshold = .01
        current_angle = self.pth
        #cmd_vel stuff
        self.send_speed(linear_speed, rotational_speed)

        while (((final_x - current_x) > threshold) or ((final_y - current_y) > threshold)):
            current_x = self.px
            current_y = self.py
            rospy.sleep(.050)

        self.send_speed(0, 0)



    def smooth_drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        init_x = self.px
        init_y = self.py
        distance_traveled = 0
        threshold = .05
        slow_speed = linear_speed/3
        medium_speed = linear_speed * 2/3

        self.send_speed(slow_speed, 0)

        while ((distance/10 - distance_traveled) > threshold):
            distance_traveled = math.sqrt((self.px - init_x)**2 + (self.py - init_y)**2)
            rospy.sleep(.050)

        self.send_speed(medium_speed, 0)

        while (((distance * 2/10) - distance_traveled) > threshold):
            distance_traveled = math.sqrt((self.px - init_x)**2 + (self.py - init_y)**2)
            rospy.sleep(.050)

        self.send_speed(linear_speed, 0)

        while (((distance * 8/10) - distance_traveled) > threshold):
            distance_traveled = math.sqrt((self.px - init_x)**2 + (self.py - init_y)**2)
            rospy.sleep(.050)

        self.send_speed(medium_speed, 0)

        while (((distance * 9/10) - distance_traveled) > threshold):
            distance_traveled = math.sqrt((self.px - init_x)**2 + (self.py - init_y)**2)
            rospy.sleep(.050)

        self.send_speed(slow_speed, 0)

        while ((distance - distance_traveled) > threshold):
            distance_traveled = math.sqrt((self.px - init_x)**2 + (self.py - init_y)**2)
            rospy.sleep(.050)


        self.send_speed(0, 0)





    def run(self):
        rospy.spin()


if __name__ == '__main__':
    Lab2().run()
