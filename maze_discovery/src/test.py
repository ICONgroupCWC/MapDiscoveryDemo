#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Int32MultiArray, Float32, Bool, String
import numpy as np

class DataCollect:
    def __init__(self):
        self.current_location = np.zeros(2)
        self.current_pose = 0
        self.block_type = ""
        rospy.Subscriber('/my_current_location', Int32MultiArray, self.location_callback)
        rospy.Subscriber('/my_pose', Int32, self.pose_callback)
        self.data_collect_pub = rospy.Publisher('/start_laser', Int32, queue_size=10)
        self.block_pub = rospy.Publisher('/block_type', String,queue_size=10)
        self.get_block()

    def location_callback(self):
        self.current_location = msg.data

    def pose_callback(self):
        self.current_pose = msg.data

    def get_block(self):
        array_block = np.array([["U", "I", "H", "L"],
                        ["L", "E", "I", "I"],
                        ["I", "E", "E", "L"],
                        ["U", "I", "I", "U"]], dtype=str)

        array_pose0 = np.array([[1, 4, 1, 4],
                                [3, 0, 4, 1],
                                [3, 0, 0, 1],
                                [4, 4, 2, 3]])
    
        array_pose1 = np.array([[4, 3, 2, 3],
                                [2, 0, 3, 4],
                                [2, 0, 0, 4],
                                [3, 3, 1, 2]])

        array_pose2 = np.array([[3, 2, 1, 2],
                                [1, 0, 2, 3],
                                [1, 0, 0, 3],
                                [2, 2, 4, 1]])

        array_pose3 = np.array([[2, 1, 2, 1],
                                [4, 0, 1, 2],
                                [4, 0, 0, 2],
                                [1, 1, 1, 4]])
        if pose == 0:
            array_pose = array_pose0
        elif pose == 1:
            array_pose = array_pose1
        elif pose == 2:
            array_pose = array_pose2
        elif pose == 3:
            array_pose = array_pose3
        
        self.block_type = array_block[self.current_location[0],self.current_location[1]] + str(array_pose[self.current_location[0],self.current_location[1]])
        self.block_pub(self.block_type)
        

    
        
if __name__ == '__main__':
    RotateRobot()
    rospy.spin()
