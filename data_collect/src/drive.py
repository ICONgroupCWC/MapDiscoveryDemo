#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
import numpy as np
from geometry_msgs.msg import Twist
from numpy import interp

move_cmd = Twist()

def callback(message: Joy):
   
    lval = round(interp(message.axes[1],[-1,1],[-0.2,0.2]),2)
    move_cmd.linear.x = lval
    move_cmd.linear.y = 0.0
    move_cmd.linear.z = 0
    
    aval = round(interp(message.axes[2],[-1,1],[-1,1]),2)
    
    move_cmd.angular.x = 0.0
    move_cmd.angular.y = 0.0
    move_cmd.angular.z = aval
    
    stt = "linear_velocity = " + str(round(message.axes[1], 2)) + " Angular_velocity = " + str(aval)
    print(stt)


def main():

    topic1 = '/joy';
    topic2 = '/cmd_vel';
        
    rospy.init_node('joy_sub',anonymous=True);
    pub = rospy.Publisher(topic2, Twist,queue_size=10);
    rospy.Subscriber(topic1,Joy, callback);
    
    while not rospy.is_shutdown():
        
        pub.publish(move_cmd)


if __name__ == '__main__':

    try:
        main()
       
    except rospy.ROSInterruptException:
        rospy.logerr('Subscriber error');
        pass;
