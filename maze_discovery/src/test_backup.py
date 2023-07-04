#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, Bool

class RotateRobot:
    def __init__(self):
        self.rotate_90_pub = rospy.Publisher('/rotate_90', Int32, queue_size=1)
        self.data_collect_pub = rospy.Publisher('/start_laser', Int32, queue_size=1)
        rospy.Subscriber('/rotate_ind', Int32, self.rotate_indicator_callback)
        rospy.Subscriber('/laser_collect_ind', Bool, self.data_collection_callback)
        rospy.Subscriber('/my_start_collect', Int32, self.start_callback)
        self.rotation_count = 0
        self.rotation_ind = False
        self.data_collection_flag = False
        # Wait for a brief moment to allow other nodes to initialize
        rospy.sleep(1.0)
        # Publish a '1' to the '/rotate_90' topic to initiate the first rotation 
        self.rotate_90_pub.publish(1)
        #self.timer = rospy.Timer(rospy.Duration(0.1),self.rotate_publisher)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.rotate_publisher)
        self.start_ind = False

    def start_callback(self, msg):
        # Check for button press event (assuming button 0)
        if msg.buttons[0] == 1:
            self.start_ind = True


    def rotate_indicator_callback(self, msg):
        if msg.data == 1:
            self.data_collect_pub.publish(1)
            self.rotation_ind = True

            # while not self.data_collection_flag:     

    def rotate_publisher(self,event):
        if (self.rotation_ind == True) and (self.data_collection_flag == True):
            self.rotation_count += 1
            if self.rotation_count < 4:
                rospy.loginfo("Rotating 90 degrees")
                self.rotate_90_pub.publish(1)
            else:
                rospy.loginfo("Finished rotating 90 degrees four times")
                self.start_ind = False
            self.data_collection_flag = False

    def data_collection_callback(self,msg):
        self.data_collection_flag = msg.data


    def run(self):
        rospy.loginfo("Starting rotation")
        self.rotate_90_pub.publish(1)
        rospy.spin()

def main():
    rospy.init_node('train_node', anonymous=True)
    rotate_robot = RotateRobot()
    rotate_robot.run()
    rospy.spin()


if __name__ == '__main__':
    main()


# import rospy
# from sensor_msgs.msg import Joy
# import numpy as np
# from geometry_msgs.msg import Twist
# from numpy import interp
# from std_msgs.msg import Int32

# rotate_ind = 0

# def callback(message):
#     global rotate_ind

#     for i in range(4):

#     if rotate_ind == 1:

    


# def main:
#     topic1 = '/rotate_ind'
#     topic2 = '/rotate_90'

#     rospy.init_node('train_node',anonymous=True);


#     rospy.Subscriber(topic1,Int32, callback);