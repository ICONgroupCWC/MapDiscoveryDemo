#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

run_strt = False

def callback(message):
    global run_strt 
    if(message.buttons[7]==1):
        run_strt = True 
    else:
        run_strt = False


def main():

    topic1 = '/joy';
    topic2 = '/my_start_infer'
    
        
    rospy.init_node('joy_sub',anonymous=True);
    rospy.Subscriber(topic1,Joy, callback);
    pub = rospy.Publisher(topic2, Bool,queue_size=10);

    while not rospy.is_shutdown():
        pub.publish(run_strt)


if __name__ == '__main__':

    try:
        main()
       
    except rospy.ROSInterruptException:
        rospy.logerr('Subscriber error');
        pass;
