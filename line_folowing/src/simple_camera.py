#!/usr/bin/env python3

import cv2
import time
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Image


move_cmd = Twist()
move_cmd2 = Twist()

run_complete = 0
prv_err = 0
stop_cmd = True

bot_run = False
raw_image = np.zeros((480,640,3), np.uint8)

def callback(x):
	global H_low,H_high,S_low,S_high,V_low,V_high
	H_low = cv2.getTrackbarPos('low H','controls')
	H_high = cv2.getTrackbarPos('high H','controls')
	S_low = cv2.getTrackbarPos('low S','controls')
	S_high = cv2.getTrackbarPos('high S','controls')
	V_low = cv2.getTrackbarPos('low V','controls')
	V_high = cv2.getTrackbarPos('high V','controls')


#cv2.namedWindow('controls',2)
#cv2.resizeWindow("controls", 550,10);


H_low = 0
H_high = 179
S_low= 0
S_high = 101
V_low= 137
V_high = 255



cv2.createTrackbar('low H','controls',0,179,callback)
cv2.createTrackbar('high H','controls',179,179,callback)

cv2.createTrackbar('low S','controls',0,255,callback)
cv2.createTrackbar('high S','controls',255,255,callback)

cv2.createTrackbar('low V','controls',0,255,callback)
cv2.createTrackbar('high V','controls',255,255,callback)


def callback2(x):
	global ob_H_low,ob_H_high,ob_S_low,ob_S_high,ob_V_low,ob_V_high
	ob_H_low = cv2.getTrackbarPos('low H','red_controls')
	ob_H_high = cv2.getTrackbarPos('high H','red_controls')
	ob_S_low = cv2.getTrackbarPos('low S','red_controls')
	ob_S_high = cv2.getTrackbarPos('high S','red_controls')
	ob_V_low = cv2.getTrackbarPos('low V','red_controls')
	ob_V_high = cv2.getTrackbarPos('high V','red_controls')

#cv2.namedWindow('red_controls',2)
#cv2.resizeWindow("red_controls", 550,10);


ob_H_low = 0
ob_H_high = 179
ob_S_low= 59
ob_S_high = 255
ob_V_low= 85
ob_V_high = 255



cv2.createTrackbar('low H','red_controls',0,179,callback2)
cv2.createTrackbar('high H','red_controls',179,179,callback2)

cv2.createTrackbar('low S','red_controls',0,255,callback2)
cv2.createTrackbar('high S','red_controls',255,255,callback2)

cv2.createTrackbar('low V','red_controls',0,255,callback2)
cv2.createTrackbar('high V','red_controls',255,255,callback2)




def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=640,
    display_height=480,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )



def line_following(hsvimg,pub):
    
    global move_cmd, stop_cmd, prv_err

    stop_cmd = False
    hsv_low = np.array([H_low, S_low, V_low], np.uint8)
    hsv_high = np.array([H_high, S_high, V_high], np.uint8)

    ob_hsv_low = np.array([ob_H_low, ob_S_low, ob_V_low], np.uint8)
    ob_hsv_high = np.array([ob_H_high, ob_S_high, ob_V_high], np.uint8)

    # crop = hsvimg[380:480,120:520]  
    # cv2.rectangle(hsvimg, (170,380), (470,480), (255,0,0), 1)
    crop = hsvimg[300:420,120:520]  
    cv2.rectangle(hsvimg, (120,300), (520,420), (255,0,0), 1)

    blank_img = np.zeros(crop.shape, np.uint8)
    blank_img2 = np.zeros(crop.shape, np.uint8)
     
    mask = cv2.inRange(crop, hsv_low, hsv_high)
    mask2 = cv2.inRange(crop, ob_hsv_low, ob_hsv_high)

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if(len(contours)>0):
        largest_contour = max(contours, key=cv2.contourArea)
        cv2.drawContours(blank_img, [largest_contour], -1, (255, 255, 255), cv2.FILLED)
        M = cv2.moments(largest_contour)
        width = mask.shape[1]
        #print(M['m00'])
        if M['m00'] > 7000:

            kp = 4
            kd = 0.8
            ki = 0

            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(mask, (cx,cy), 10, (255,0,0), 2)
            
            err = cx - width/2
            dif = err-prv_err
            
            pid = err*kp + kd*dif 
      
            move_cmd.linear.x = 0.1
            angular_speed = -float(pid) / 100

            if(angular_speed>=0.25):
                angular_speed = 0.25

            elif(angular_speed<=-0.25):
                angular_speed = -0.25

            move_cmd.angular.z = angular_speed

            pub.publish(move_cmd)
            prv_err = err
            
        
        # else:
        #     print("line following fail")
            
    # else:
    #     print("line following fail.check the hsv value")
        

    contours2, hierarchy2 = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if(len(contours2)>0):
        largest_contour2 = max(contours2, key=cv2.contourArea)
        cv2.drawContours(blank_img2, [largest_contour2], -1, (255, 255, 255), cv2.FILLED)
        M2 = cv2.moments(largest_contour2)
        #print(M2['m00'])
        if(M2['m00']>44000):
            move_cmd.linear.x = 0
            move_cmd.angular.z = 0
            pub.publish(move_cmd)
            
            stop_cmd = True
            # print("robot stop")

    # cv2.imshow("crop",mask)
    # cv2.imshow("red_crop",mask2)
    # cv2.imshow("contour",blank_img)
    # cv2.imshow("red contour",blank_img2)
    # cv2.imshow("Hsv",hsvimg)
    cv2.waitKey(1)
    return hsvimg

   

def run_distance(pub):
    global move_cmd2
    move_cmd2.linear.x = 0.05    
    move_cmd2.angular.z = 0.0
    start_time = rospy.Time.now().to_sec()

    while (rospy.Time.now().to_sec() - start_time) < 2.0: 
        pub.publish(move_cmd2)  
        time.sleep(0.01)  

    move_cmd2.linear.x = 0.0
    move_cmd2.angular.z = 0.0
    pub.publish(move_cmd2)

def bot_callback(message):
    global bot_run 
    
    if(message.data==True):
        bot_run = True 
    else:
        bot_run = False   
    #print(bot_run)


def image_callback(msg):
    global raw_image
    raw_image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))

def main():
        
    topic1 = '/cmd_vel'
    topic2 = '/my_start_infer'
    topic3 = '/my_forward'
    topic4 = '/csi_cam_0/image_raw'
    
    # video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

    # if(video_capture.isOpened()!=1):
    #     print("Error: Unable to open camera")


    rospy.init_node('Camera_node',anonymous=True);
    #bot_run = False
    pub = rospy.Publisher(topic1, Twist,queue_size=10);
    rospy.Subscriber(topic2,Bool, bot_callback);
    pub2 = rospy.Publisher(topic3, Bool,queue_size=10);
    rospy.Subscriber(topic4,Image, image_callback);

    robot_run = True
    line_follow = False
    frst_red = False
    line_follow2 = False

    status = True
    while not rospy.is_shutdown():
        global bot_run
        global raw_image
        print(bot_run)
           
	#ret_val2, frame = video_capture.read()
##------------------------------------
        if(bot_run==True):

            while(robot_run==True):
                status = False
                for j in range(10):
                    pub2.publish(status)
                print("stage 1")
                run_distance(pub)
                # for i in range(100):
                #     ret_val2, frame = video_capture.read()
                line_follow = True
                bot_run = False
                robot_run = False

                

            while(line_follow==True):
                status = False
                for j in range(10):
                    pub2.publish(status)
                print("stage 2")
                #ret_val, frame = video_capture.read()
                hsv_img = cv2.cvtColor(raw_image, cv2.COLOR_BGR2HSV)
                hsv_img = line_following(hsv_img,pub)
                # cv2.imshow("csi view2", frame)
                # cv2.imshow("hsv image2",hsv_img)
                if(stop_cmd==True):
                    print("stop first red line")
                    frst_red = True
                    line_follow = False
    
            while(frst_red==True):
                status = False
                for j in range(10):
                    pub2.publish(status)
                print("stage 3") 
                run_distance(pub)
                # for i in range(100):
                #     ret_val3, frame3 = video_capture.read()
                line_follow2 = True
                frst_red = False

            while(line_follow2==True):
                print("stage 4")
                status = False
                for j in range(10):
                    pub2.publish(status)
                hsv_img = cv2.cvtColor(raw_image, cv2.COLOR_BGR2HSV)
                hsv_img = line_following(hsv_img,pub)
                # cv2.imshow("csi view", frame)
                # cv2.imshow("hsv image",hsv_img)
                if(stop_cmd==True):
                    status = True
                    for j in range(10):
                        pub2.publish(status)
                    print("completely stop")
                    robot_run = True
                    bot_run = False
                    line_follow2 = False

                    
##------------------------------------
        
        else:
            print("stop")
        
        if cv2.waitKey(10) & 0xFF == 27 or cv2.waitKey(10) & 0xFF == ord('q'):
            break

    #video_capture.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
