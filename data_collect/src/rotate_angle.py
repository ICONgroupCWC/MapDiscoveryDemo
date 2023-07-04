#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Int32

class RotateRobot:
    def __init__(self):
        rospy.init_node('rotate_robot_node', anonymous=True)

        self.target_angle = None
        self.current_angle = None
        self.starting_angle = None
        self.error_carried_forward = 0
        self.initial_rotation = True

        self.angle_tolerance = self.deg2rad(1)  # Set the angle tolerance to 1 degree
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.initialize_starting_angle()
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.rotate_command_sub = rospy.Subscriber('/rotate_90', Int32, self.rotate_command_callback)

        self.pid_controller1 = PIDController(3, 0, 1)
        self.pid_controller2 = PIDController(3, 0, 1)  # Adjust the PID gains as needed
        self.pid_controller = None

        self.is_rotating = False
        self.rotation_start_time = None
        self.rotation_timeout = rospy.Duration(5)  # Timeout duration of 10 seconds

        self.timer = rospy.Timer(rospy.Duration(0.1), self.rotate_robot)
        self.indicate = False

        self.rotate_fin_ind = rospy.Publisher('/rotate_ind', Int32,queue_size=10)

        self.my_pose = 0
        self.pose_pub = rospy.Publisher('/my_pose', Int32, queue_size=10)
        self.pose_pub.publish(self.my_pose)
        
    def initialize_starting_angle(self):
        rospy.wait_for_message('/odom', Odometry)  # Wait for the first Odometry message to arrive
        orientation = rospy.wait_for_message('/odom', Odometry).pose.pose.orientation
        _, _, self.starting_angle = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    

    def rotate_command_callback(self, message):
        # print(message.data)
        if message.data == 1 or message.data == -1 or message.data == 2:
            if not self.is_rotating:
                if message.data == 1:
                    self.pid_controller = self.pid_controller1
                    self.my_pose = (self.my_pose - 1) % 4
                   
                elif message.data == -1:
                    self.pid_controller = self.pid_controller2
                    self.my_pose = (self.my_pose + 1) % 4

                if message.data == 2:
                    self.pid_controller = self.pid_controller1
                    self.my_pose = (self.my_pose - 2) % 4

                
                self.is_rotating = True
                self.target_angle = self.normalize_angle(self.current_angle + self.deg2rad(90 * message.data))+self.error_carried_forward  # Rotate 90 degrees (clockwise or counterclockwise)
                self.rotation_start_time = rospy.Time.now()  # Start the rotation timer
                rospy.loginfo("Starting rotation")
                
            else:
                rospy.loginfo("Already rotating. Please wait.")

    def odom_callback(self, msg):
        orientation = msg.pose.pose.orientation
        _, _, self.current_angle = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

    def rotate_robot(self, event):
        if self.target_angle is None:
            return

        if self.cmd_vel_pub.get_num_connections() > 0:
            error_angle = self.normalize_angle(self.target_angle - self.current_angle)
            rotation_speed = self.pid_controller.update(error_angle)
            angular_speed = self.clamp_rotation_speed(rotation_speed)

            twist = Twist()
            twist.angular.z = angular_speed
            self.cmd_vel_pub.publish(twist)

            if abs(error_angle) < self.angle_tolerance:
                self.cmd_vel_pub.publish(Twist())  # Publish an empty Twist message to stop the robot's rotation
                self.is_rotating = False
                self.rotation_start_time = None  # Reset the rotation timer
                self.save_error()  # Save the error angle
                self.target_angle = None
                rospy.loginfo("Reached the target angle")
                self.rotate_fin_ind.publish(1)
                self.pose_pub.publish(self.my_pose)
                

            elif self.rotation_start_time is not None and (rospy.Time.now() - self.rotation_start_time) >= self.rotation_timeout:
                self.cmd_vel_pub.publish(Twist())  # Publish an empty Twist message to stop the robot's rotation
                self.is_rotating = False
                self.rotation_start_time = None  # Reset the rotation timer
                self.save_error()  # Save the error angle
                self.target_angle = None
                rospy.loginfo("Reached the target angle with error (Timeout):{}".format(self.error_carried_forward))
                self.rotate_fin_ind.publish(1)
                self.pose_pub.publish(self.my_pose)

    def save_error(self):
        self.error_carried_forward = self.target_angle-self.current_angle
        rospy.loginfo("Saved error angle: {}".format(self.error_carried_forward))

    def deg2rad(self, angle_degrees):
        return angle_degrees * (math.pi / 180.0)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def clamp_rotation_speed(self, speed):
        max_angular_speed = rospy.get_param('/jetbot/max_angular_speed', 2)
        return max(min(speed, max_angular_speed), -max_angular_speed)

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0.0
        self.integral = 0.0

    def update(self, error):
        self.integral += error
        derivative = error - self.last_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.last_error = error
        return output

if __name__ == '__main__':
    RotateRobot()
    rospy.spin()
