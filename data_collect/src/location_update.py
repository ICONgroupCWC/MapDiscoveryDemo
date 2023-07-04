#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, Int32MultiArray, Bool

class MazeTracker:
    def __init__(self):
        # self.current_location = IntList()
        self.current_location = (0, 0)
        self.my_pose = 0
        self.location_pub = rospy.Publisher('/my_current_location', Int32MultiArray, queue_size=10)
        self.updating_flag = True
        rospy.Subscriber('/my_pose', Int32, self.pose_callback)
        # rospy.Subscriber('/my_forward', Int32, self.forward_callback)
        rospy.Subscriber('/my_forward', Bool, self.forward_callback)
        # self.line_follow_start = rospy.Publisher('/my_start_infer', Bool, queue_size=10)
    

    def pose_callback(self, msg):
        self.my_pose = msg.data

    def forward_callback(self, msg):
        # print(msg.data)
        if msg.data == True:
            if self.updating_flag == True:
                self.move_forward()
                # print(self.updating_flag)
                self.updating_flag = False
                
        else:
            self.updating_flag = True

    def move_forward(self):
        print("My direction is: " + str(self.my_pose))
        x, y = self.current_location
        if self.my_pose == 0:
            self.current_location = (x,y+1)
        elif self.my_pose == 1:
            self.current_location = (x+1,y)
        elif self.my_pose == 2:
            self.current_location = (x,y-1)
        elif self.my_pose == 3:
            self.current_location = (x-1,y)
        rospy.loginfo("Updated current location: {}".format(self.current_location))
        self.publish_location()

    def publish_location(self):
        location_msg = Int32MultiArray(data=self.current_location)
        self.location_pub.publish(location_msg)
        #publish to maze navigator
        # self.line_follow_start.publish(True)

if __name__ == '__main__':
    rospy.init_node('maze_tracker_node', anonymous=True)
    maze_tracker = MazeTracker()
    rospy.spin()
