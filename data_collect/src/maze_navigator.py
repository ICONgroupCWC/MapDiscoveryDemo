#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Bool

class MazeNavigator:
    def __init__(self):
        self.block_type = None
        self.rotation_ind = False
        self.decision = False
        self.block_received = False
        rospy.init_node('maze_navigator_node', anonymous=True)
        rospy.Subscriber('/block_type_actual', String, self.block_type_callback)
        rospy.Subscriber('/rotate_ind', Int32, self.rotate_indicator_callback)
        rospy.Subscriber('/decision_start', Bool, self.decision_start_callback)
        self.move_ind = rospy.Publisher('/my_start_infer', Bool,queue_size=1)
        self.rotate_angle_pub = rospy.Publisher('/rotate_90', Int32, queue_size=1)

    def decision_start_callback(self,msg):
        if msg.data == True:
            print(msg.data)
            self.decision = True
            # print("Taking Decision for the blocktype: " + self.block_type)
            
            # self.take_decision()

    def block_type_callback(self, msg):
        self.block_type = msg.data
        rospy.loginfo("Current block type: {}".format(self.block_type))
        rospy.sleep(5)
        print("Waiting for take decision")
        if self.decision:
            self.take_decision()
            print("Taking Decision for the blocktype: " + self.block_type)

    def take_decision(self):
        # Make decisions based on block type
        if self.block_type.startswith('U'):
            self.handle_U_shape()
        elif self.block_type.startswith('L'):
            self.handle_L_shape()
        elif self.block_type.startswith('I'):
            self.handle_I_shape()
        elif self.block_type.startswith('H'):
            self.handle_H_shape()
        elif self.block_type == 'E':
            self.handle_empty_space()
        else:
            rospy.logwarn("Unknown block type: {}".format(self.block_type))

    def handle_U_shape(self):
        # Perform actions for U shape
        rospy.loginfo("Performing actions for U shape")
        if self.block_type.endswith('1'):
            self.go_forward()

        elif self.block_type.endswith('2'):
            self.rotate_cw_90_and_move_forward()

        elif self.block_type.endswith('3'):
            self.rotate_180_and_move_forward()

        elif self.block_type.endswith('4'):
            self.rotate_ccw_90_and_move_forward()

        else:
            rospy.logwarn("Unknown block type: {}".format(self.block_type))

    def handle_L_shape(self):
        # Perform actions for L shape
        rospy.loginfo("Performing actions for L shape")
        if self.block_type.endswith('1'):
            self.rotate_ccw_90_and_move_forward()

        elif self.block_type.endswith('2'):
            action = random.choice([self.go_forward, self.rotate_ccw_90_and_move_forward])
            action()
            # go forward or -90 and go forward

        elif self.block_type.endswith('3'):
            action = random.choice([self.go_forward, self.rotate_cw_90_and_move_forward])
            action()
            # go forward or 90 and go forward

        elif self.block_type.endswith('4'):
            self.rotate_cw_90_and_move_forward()

        else:
            rospy.logwarn("Unknown block type: {}".format(self.block_type))


    def handle_I_shape(self):
        # Perform actions for I shape
        rospy.loginfo("Performing actions for I shape")
        if self.block_type.endswith('1'):
            action = random.choice([self.rotate_ccw_90_and_move_forward, self.rotate_cw_90_and_move_forward])
            action()
            # turn -90 and go forward or turn 90 and go forward

        elif self.block_type.endswith('2'):
            action = random.choice([self.go_forward, self.rotate_ccw_90_and_move_forward])
            action()
            # go forward or -90 and go forward


        elif self.block_type.endswith('3'):
            action = random.choice([self.go_forward,self.rotate_cw_90_and_move_forward, self.rotate_ccw_90_and_move_forward])
            action()
            # go forward or turn 90 and go forward or turn -90 and go forward

        elif self.block_type.endswith('4'):
            action = random.choice([self.go_forward, self.rotate_cw_90_and_move_forward])
            action()
            # go forward or 90 and go forward

        else:
            rospy.logwarn("Unknown block type: {}".format(self.block_type))

    def handle_H_shape(self):
        # Perform actions for H shape
        rospy.loginfo("Performing actions for H shape")
        if self.block_type.endswith('1'):
            self.go_forward()

        elif self.block_type.endswith('2'):
            action = random.choice([self.rotate_ccw_90_and_move_forward, self.rotate_cw_90_and_move_forward])
            action()
            # turn -90 and go forward or turn 90 and go forward

        else:
            rospy.logwarn("Unknown block type: {}".format(self.block_type))

    def handle_empty_space(self):
        # Perform actions for empty space
        rospy.loginfo("Performing actions for empty space")

        if self.block_type.startswith('E'):
            action = random.choice([self.go_forward,self.rotate_cw_90_and_move_forward, self.rotate_ccw_90_and_move_forward])
            action()
            # go forward or turn 90 and go forward or turn -90 and go forward

        else:
            rospy.logwarn("Unknown block type: {}".format(self.block_type))

    def rotate_indicator_callback(self, msg):
        if msg.data == 1:
            self.rotation_ind = True

    # Action Space
    def go_forward(self):
        rospy.loginfo("Moving forward")
        # self.move_ind.publish(True)
        for _ in range(10):
            self.move_ind.publish(True)
            rospy.sleep(0.1)
        print("Move Forward in Line")
        self.decision = False
        
    def rotate_cw_90_and_move_forward(self):
        rospy.loginfo("Rotate 90 and Moving forward")
        self.rotate_angle_pub.publish(-1)
        # Wait for the rotation to complete
        self.rotation_ind = False
        while not self.rotation_ind:
            if self.rotation_ind: 
                self.rotation_ind = True
            rospy.sleep(0.1)
        # self.move_ind.publish(True)
        self.decision = False
        for _ in range(5):
            self.move_ind.publish(True)
            rospy.sleep(0.1)
        print("Rotate 90 and Moving forward")
        # self.decision = False

    def rotate_ccw_90_and_move_forward(self):
        rospy.loginfo("Rotate -90 and Moving forward")
        self.rotate_angle_pub.publish(1)
        # Wait for the rotation to complete
        self.rotation_ind = False
        while not self.rotation_ind:
            if self.rotation_ind: 
                self.rotation_ind = True
            rospy.sleep(0.1)
        self.decision = False
        for _ in range(5):
            self.move_ind.publish(True)
            rospy.sleep(0.1)
        # self.move_ind.publish(True)
        print("Rotate -90 and Moving forward")
        # self.decision = False

    def rotate_180_and_move_forward(self):
        rospy.loginfo("Rotate 180 and Moving forward")
        self.rotate_angle_pub.publish(2)
        # Wait for the rotation to complete
        self.rotation_ind = False
        while not self.rotation_ind:
            if self.rotation_ind: 
                self.rotation_ind = True
            rospy.sleep(0.1)
        # self.move_ind.publish(True)
        self.decision = False
        for _ in range(5):
            self.move_ind.publish(True)
            rospy.sleep(0.1)
        print("Rotate 180 and Moving forward")
        # self.decision = False


if __name__ == '__main__':
    MazeNavigator()
    rospy.spin()
