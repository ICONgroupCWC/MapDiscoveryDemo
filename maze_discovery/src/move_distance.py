#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import tf
from math import sqrt, pow
from std_msgs.msg import Bool

class MoveRobot:
    def __init__(self):
        rospy.init_node('move_robot_node', anonymous=True)

        self.target_distance = None
        self.current_distance = None
        self.distance_tolerance = 0.02  # Adjust the tolerance if necessary

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.move_command_sub = rospy.Subscriber('/move_40', Int32, self.move_command_callback)

        self.timer = None
        self.initial_position = None

    def move_command_callback(self, message):
        if message.data == 1:
            self.target_distance = 0.4  # Move 40 cm
            self.start_movement()
        elif message.data == -1:
            self.target_distance = -0.4  # Move -40 cm
            self.start_movement()

    def start_movement(self):
        if self.target_distance is None:
            return

        self.timer = rospy.Timer(rospy.Duration(0.1), self.move_robot)
        self.initial_position = self.get_robot_position()

    def stop_movement(self):
        if self.timer is not None:
            self.timer.shutdown()
            self.cmd_vel_pub.publish(Twist())  # Publish an empty Twist message to stop the robot's movement

    def move_robot(self, event):
        if self.target_distance is None:
            return

        if self.cmd_vel_pub.get_num_connections() > 0:
            move_cmd = Twist()
            move_cmd.linear.x = 0.3  # Adjust the speed if necessary

            self.cmd_vel_pub.publish(move_cmd)

            current_position = self.get_robot_position()
            distance_traveled = abs(current_position - self.initial_position)
            self.current_distance = distance_traveled

            if self.current_distance >= abs(self.target_distance):
                self.stop_movement()
                rospy.loginfo("Reached the target distance")

    def get_robot_position(self):
        try:
            with tf.TransformListener() as tf_listener:
                tf_listener.waitForTransform('/odom', '/base_footprint', rospy.Time(0), rospy.Duration(1.0))
                (trans, rot) = tf_listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
                position = sqrt(pow(trans[0], 2) + pow(trans[1], 2))
                return position
        except (tf.LookupException, tf.ExtrapolationException) as e:
            rospy.logwarn("TF Exception occurred while retrieving position: %s" % str(e))
            return None

if __name__ == '__main__':
    MoveRobot()
    rospy.spin()
