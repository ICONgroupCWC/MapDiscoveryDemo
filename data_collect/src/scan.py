#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32,Bool
import os
import glob
data_stream = []

flag = False
map_array = ['U2','U3','U4','U1','I1','I2','I3','I4','H2','H1','H2','H1','L1','L2','L3','L4','E', 'E','E','E']
i = 0
np_array = []
topic1 = '/laser_collect_ind'
laser_ind_pub = rospy.Publisher(topic1, Bool,queue_size=10)


def callback_laser(msg):
    global data_stream, flag, i,map_array
    np_array = np.asarray(msg.ranges)
    if flag==True:
            data_stream.append(np_array)
            # print(len(data_stream))
            if len(data_stream) == 50:
                data_stream1 = np.asarray(data_stream)
                # np.save('/home/jetbot/numpy_arrays/'+map_array[i]+'.npy',data_stream1)
                filename = '/home/jetbot/numpy_arrays/' + map_array[i] + '.npy'
            
                if os.path.isfile(filename):
                    existing_data = np.load(filename)
                    data_stream1 = np.concatenate((existing_data, data_stream1), axis=0)
            
                np.save(filename, data_stream1)

                i += 1
                flag = False
                laser_ind_pub.publish(True)
                data_stream = []
                print('Finished Collecting ' + str(data_stream1.shape[0]) + ' Lidar Data samples in shape ' + map_array[i-1])

def callback_read(msg):
    global flag
    if (msg.data == 1):
        flag = True
        print('Start Collecting Lidar Data')

def callback_pose(msg):
    global pose
    pose = msg.data
    

def delete_npy_files(directory):
    npy_files = glob.glob(directory + '*.npy')
    
    for file in npy_files:
        os.remove(file)
    
    print('Deleted {} .npy files'.format(len(npy_files)))

    
def main():
    
    rospy.init_node('read_lidar')
    rospy.Subscriber('/scan', LaserScan, callback_laser)
    rospy.Subscriber('/start_laser', Int32, callback_read)
    rospy.Subscriber('/my_pose', Int32, callback_read)
    # delete_npy_files('/home/jetbot/numpy_arrays/')
    rospy.spin()
    # while not rospy.is_shutdown():
    #     laser_ind_pub.publish(flag)       

if __name__ == '__main__':
    main()
