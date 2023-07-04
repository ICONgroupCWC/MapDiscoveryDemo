#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
import numpy as np
from geometry_msgs.msg import Twist
from numpy import interp
from std_msgs.msg import Int32, Bool


rotate_ind = 0
move_ind = 0
# laser_start = 0
start_collect = 0
move_cmd = Twist()
b_val = 0
y_val = False
start_line = False

def callback(message: Joy):
    global rotate_ind
    global move_ind
    # global laser_start
    global start_collect
    global b_val
    global y_val
    global start_line
   #reading the joystick buttons and assign them to variables
    
    b_val = message.buttons[1]
    x_val = message.buttons[3]
    if message.buttons[4] == 1:
        y_val = True
    else:
        y_val = False

    if message.buttons[11] == 1:
        start_line = True
    else:
        start_line = False


    # laser_start = message.buttons[11]
    start_collect = message.buttons[0]
    rotate_ind = int(message.axes[6])
    move_ind = int(message.axes[7])

    lval = round(interp(message.axes[1],[-1,1],[-0.2,0.2]),2)
    move_cmd.linear.x = lval
    move_cmd.linear.y = 0.0
    move_cmd.linear.z = 0
    
    aval = round(interp(message.axes[2],[-1,1],[-1,1]),2)
    move_cmd.angular.x = 0.0
    move_cmd.angular.y = 0.0
    move_cmd.angular.z = aval

def main():
    topic1 = '/joy'
    topic2 = '/rotate_only'
    topic3 = '/move_40'
    topic4 = '/cmd_vel'
    # topic5 = '/start_laser'
    topic6 = '/my_start_collect'
    topic7= '/my_forward'
        
    rospy.init_node('main_init_node',anonymous=True)

    #Publisher node to rotate 90 degrees
    pub_rotate_ind = rospy.Publisher(topic2, Int32,queue_size=10)
    #Publisher node to move 40cm
    pub_move_ind = rospy.Publisher(topic3, Int32,queue_size=10)
    #Publishing the movements from the remote
    pub_cmd = rospy.Publisher(topic4, Twist,queue_size=10)
    #Publisher to start the LaserScan
    # pub_laser_start = rospy.Publisher(topic5, Int32,queue_size=1);
    #Publisher to start the Collecting data
    pub_start_collect = rospy.Publisher(topic6, Int32,queue_size=10)
    #Dummy publisher to indicate the move fwd
    pub_dummy_move = rospy.Publisher(topic7, Bool,queue_size=10)
    #Publisher to move in line
    move_line_pub = rospy.Publisher('/move_line', Bool, queue_size = 10)


    rospy.Subscriber(topic1,Joy, callback)

    while not rospy.is_shutdown():
        pub_move_ind.publish(move_ind)
        
        # pub_laser_start.publish(laser_start)
        pub_start_collect.publish(start_collect)

        if b_val == 1:
            pub_cmd.publish(move_cmd)
            pub_rotate_ind.publish(rotate_ind)
            pub_dummy_move.publish(y_val)
            move_line_pub.publish(start_line)
        

if __name__ == '__main__':

    try:
        main()
       
    except rospy.ROSInterruptException:
        rospy.logerr('Subscriber error')
        pass
