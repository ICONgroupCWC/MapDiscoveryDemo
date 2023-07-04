#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import time 
from collections import Counter

from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String


robot_direction = 0
robotpos = (0,0)
blocktype = "E"

Red = (0,0,255)
Green = (0,255,0)
Blue = (255,0,0)
white = (255,255,255)

center_location = []

block_update = True

def draw_robot_location(im,position,direction):
    
    start_point = np.asarray(position)
    end_point = np.array((0,0))
    if(direction==0):
        end_point[0] =  start_point[0]
        end_point[1] =  start_point[1]-20
        
    if(direction==1):
        end_point[0] =  start_point[0]+20
        end_point[1] =  start_point[1]
        
    if(direction==2):
        end_point[0] =  start_point[0]
        end_point[1] =  start_point[1]+20
        
    if(direction==3):
        end_point[0] =  start_point[0]-20
        end_point[1] =  start_point[1]
 
    im = cv2.arrowedLine(im, start_point,end_point, (0,0,255), 1,tipLength = 0.5)
    
    return im
 

def Point_L(position,L_shape):
        
    point1 = np.array((0,0))
    point2 = np.array((0,0))
    point3 = np.array((0,0))
    
    if(L_shape==0):
        point1[0] = position[0]-50
        point1[1] = position[1]-50
        point2[0] = position[0]+50
        point2[1] = position[1]-50
        point3[0] = position[0]+50
        point3[1] = position[1]+50
        
    elif(L_shape==1):
        point1[0] = position[0]+50
        point1[1] = position[1]-50
        point2[0] = position[0]+50
        point2[1] = position[1]+50
        point3[0] = position[0]-50
        point3[1] = position[1]+50
            
    elif(L_shape==2):
        point1[0] = position[0]+50
        point1[1] = position[1]+50
        point2[0] = position[0]-50
        point2[1] = position[1]+50
        point3[0] = position[0]-50
        point3[1] = position[1]-50    
        
    elif(L_shape==3):
        point1[0] = position[0]-50
        point1[1] = position[1]+50
        point2[0] = position[0]-50
        point2[1] = position[1]-50
        point3[0] = position[0]+50
        point3[1] = position[1]-50 
    
    return point1,point2,point3

def Point_I(position,I_shape):
    
    point1 = np.array((0,0))
    point2 = np.array((0,0)) 
    
    if(I_shape==0):
        point1[0] = position[0]-50
        point1[1] = position[1]-50
        point2[0] = position[0]+50
        point2[1] = position[1]-50
        
    elif(I_shape==1):
        point1[0] = position[0]+50
        point1[1] = position[1]-50
        point2[0] = position[0]+50
        point2[1] = position[1]+50
        
    elif(I_shape==2):
        point1[0] = position[0]+50
        point1[1] = position[1]+50
        point2[0] = position[0]-50
        point2[1] = position[1]+50
        
    elif(I_shape==3):
        point1[0] = position[0]-50
        point1[1] = position[1]+50
        point2[0] = position[0]-50
        point2[1] = position[1]-50
        
    return point1,point2
      

def Point_H(position,H_shape): 
    point1 = np.array((0,0))
    point2 = np.array((0,0)) 
    point3 = np.array((0,0))
    point4 = np.array((0,0))
    
    if(H_shape==0):
        point1[0] = position[0]-50
        point1[1] = position[1]-50
        point2[0] = position[0]-50
        point2[1] = position[1]+50 
        point3[0] = position[0]+50
        point3[1] = position[1]-50
        point4[0] = position[0]+50
        point4[1] = position[1]+50     
        
    elif(H_shape==1):
        point1[0] = position[0]+50
        point1[1] = position[1]-50
        point2[0] = position[0]-50
        point2[1] = position[1]-50 
        point3[0] = position[0]+50
        point3[1] = position[1]+50
        point4[0] = position[0]-50
        point4[1] = position[1]+50 
        
    return point1, point2,point3,point4
    

def Point_U(position,U_shape):
    point1 = np.array((0,0))
    point2 = np.array((0,0)) 
    point3 = np.array((0,0))
    point4 = np.array((0,0))
    
    if(U_shape==0):
        point1[0] = position[0]-50
        point1[1] = position[1]-50
        point2[0] = position[0]-50
        point2[1] = position[1]+50 
        point3[0] = position[0]+50
        point3[1] = position[1]+50
        point4[0] = position[0]+50
        point4[1] = position[1]-50 
        
    elif(U_shape==1):
        point1[0] = position[0]+50
        point1[1] = position[1]-50
        point2[0] = position[0]-50
        point2[1] = position[1]-50 
        point3[0] = position[0]-50
        point3[1] = position[1]+50
        point4[0] = position[0]+50
        point4[1] = position[1]+50
        
    elif(U_shape==2):
        point1[0] = position[0]+50
        point1[1] = position[1]+50
        point2[0] = position[0]+50
        point2[1] = position[1]-50 
        point3[0] = position[0]-50
        point3[1] = position[1]-50
        point4[0] = position[0]-50
        point4[1] = position[1]+50 
        
    elif(U_shape==3):
        point1[0] = position[0]-50
        point1[1] = position[1]+50
        point2[0] = position[0]+50
        point2[1] = position[1]+50 
        point3[0] = position[0]+50
        point3[1] = position[1]-50
        point4[0] = position[0]-50
        point4[1] = position[1]-50 
        
    return point1, point2,point3,point4
        

def shift_array(arr,n):
    new_array = arr[n:] + arr[:n] 
    return new_array


def draw_L(img,direction,position,L_shape):
    shape_idx = [0,1,2,3]
    
    pt1 = np.array((0,0))
    pt2 = np.array((0,0))
    pt3 = np.array((0,0))
    
    if(direction==0):
        pt1,pt2,pt3 = Point_L(position,shape_idx[L_shape])
        
    elif(direction==1):
        new_arr = shift_array(shape_idx,1)
        pt1,pt2,pt3 = Point_L(position,new_arr[L_shape])
        
    elif(direction==2):
        new_arr = shift_array(shape_idx,2)
        pt1,pt2,pt3 = Point_L(position,new_arr[L_shape])

        
    elif(direction==3):
        new_arr = shift_array(shape_idx,3)
        pt1,pt2,pt3 = Point_L(position,new_arr[L_shape])
        
    cv2.line(img,pt1,pt2,(255,255,255),3)
    cv2.line(img,pt2,pt3,(255,255,255),3)

    return img
    
def draw_I(img,direction,position,I_shape):
    
    shape_idx = [0,1,2,3]
    
    pt1 = np.array((0,0))
    pt2 = np.array((0,0))
    
    if(direction==0):
        pt1,pt2 = Point_I(position,shape_idx[I_shape])
        
    elif(direction==1):
        new_arr = shift_array(shape_idx,1)
        pt1,pt2 = Point_I(position,new_arr[I_shape])
        
    elif(direction==2):
        new_arr = shift_array(shape_idx,2)
        pt1,pt2 = Point_I(position,new_arr[I_shape])

        
    elif(direction==3):
        new_arr = shift_array(shape_idx,3)
        pt1,pt2 = Point_I(position,new_arr[I_shape])
        
    cv2.line(img,pt1,pt2,(255,255,255),3)

    return img

def draw_H(img,direction,position,H_shape):
    shape_idx = [0,1]
    
    pt1 = np.array((0,0))
    pt2 = np.array((0,0))
    pt3 = np.array((0,0))
    pt4 = np.array((0,0))
    
    if(direction==0 or direction==2):
        pt1,pt2,pt3,pt4 = Point_H(position,shape_idx[H_shape])
        
    elif(direction==1 or direction==3):
        new_arr = shift_array(shape_idx,1)
        pt1,pt2,pt3,pt4 = Point_H(position,new_arr[H_shape])
        

        
    cv2.line(img,pt1,pt2,(255,255,255),3)
    cv2.line(img,pt3,pt4,(255,255,255),3)
    return img    


def draw_U(img,direction,position,U_shape):
    
    shape_idx = [0,1,2,3]
    
    pt1 = np.array((0,0))
    pt2 = np.array((0,0))
    pt3 = np.array((0,0))
    pt4 = np.array((0,0))
    
    if(direction==0):
        pt1,pt2,pt3,pt4 = Point_U(position,shape_idx[U_shape])
        
    elif(direction==1):
        new_arr = shift_array(shape_idx,1)
        pt1,pt2,pt3,pt4 = Point_U(position,new_arr[U_shape])
        
    elif(direction==2):
        new_arr = shift_array(shape_idx,2)
        pt1,pt2,pt3,pt4 = Point_U(position,new_arr[U_shape])

        
    elif(direction==3):
        new_arr = shift_array(shape_idx,3)
        pt1,pt2,pt3,pt4 = Point_U(position,new_arr[U_shape])
        
    cv2.line(img,pt1,pt2,(255,255,255),3)
    cv2.line(img,pt2,pt3,(255,255,255),3)
    cv2.line(img,pt3,pt4,(255,255,255),3)

    return img


def draw_block(img,dir,r_pos,b_type):
    if(b_type=="U1"):
        img = draw_U(img,dir,r_pos,0)
    
    elif(b_type=="U2"):
        img = draw_U(img,dir,r_pos,1)
    
    elif(b_type=="U3"):
        img = draw_U(img,dir,r_pos,2)
    
    elif(b_type=="U4"):
        img = draw_U(img,dir,r_pos,3)
        
    elif(b_type=="L1"):
        img = draw_L(img,dir,r_pos,0)
    
    elif(b_type=="L2"):
        img = draw_L(img,dir,r_pos,1)
    
    elif(b_type=="L3"):
        img = draw_L(img,dir,r_pos,2)
    
    elif(b_type=="L4"):
        img = draw_L(img,dir,r_pos,3)
        
    elif(b_type=="I1"):
        img = draw_I(img,dir,r_pos,0)
    
    elif(b_type=="I2"):
        img = draw_I(img,dir,r_pos,1)
    
    elif(b_type=="I3"):
        img = draw_I(img,dir,r_pos,2)
    
    elif(b_type=="I4"):
        img = draw_I(img,dir,r_pos,3)
        
    elif(b_type=="H1"):
        img = draw_H(img,dir,r_pos,0)
    
    elif(b_type=="H2"):
        img = draw_H(img,dir,r_pos,0)
        
    return img
        


def callback1(msg):
    global robot_direction
    robot_direction = msg.data

def callback2(msg):
    global robotpos
    robotpos = msg.data
    
def callback3(msg):
    global blocktype ,block_update
    blocktype = msg.data   
    block_update = False 
        


def find_repeated_element(arr):
    element_counts = Counter(arr)
    for element, count in element_counts.items():
        if count > 1:
            return element
    return None


def main():
    
    topic1 = '/my_pose'
    topic2 = '/my_current_location'
    topic3 = '/block_type'
      
    blank_img = np.zeros((640,640,3), np.uint8)
    
    corner1 = (120,120)
    corner2 = (120,520)
    corner3 = (520,520)
    corner4 = (520,120)
    #cv2.rectangle(blank_img,corner1,corner3,white,3)
    
    for i in range(4):
        raw = []
        for j in range(4):
            x = 170 + j*100
            y = 470 - i*100
            center = (x,y)
            raw.append(center)
            cv2.circle(blank_img,center, 1, (255,255,255))
                
        center_location.append(raw)
    
    
    rospy.init_node('draw_maze',anonymous=True);
    
    rospy.Subscriber(topic1,Int32, callback1);
    rospy.Subscriber(topic2,Int32MultiArray, callback2);
    rospy.Subscriber(topic3,String, callback3);
    
    cor = True
    draw_location = []
    while not rospy.is_shutdown():
        
        blank_img2 = np.zeros((640,640,3), np.uint8)
        
        
        # r_pos = center_location[robotpos[0]][robotpos[1]]
        # dir = robot_direction
        # b_type = blocktype
        
        r_pos = center_location[robotpos[1]][robotpos[0]]
        dir = robot_direction
        b_type = blocktype
        
        img2 = draw_robot_location(blank_img2,r_pos,dir)
        
        if(cor==True and block_update==False):
            draw_location.append(r_pos)
            blck_type = []
            for i in range(30):
                blck_type.append(b_type)
                
            repeated_element = find_repeated_element(blck_type)
            print(repeated_element)
            blank_img = draw_block(blank_img,dir,r_pos,repeated_element)
            cor = False
            

        
        
        elif(cor==False):
            print(len(draw_location))
            chck = True
            for i in range(len(draw_location)):
                if(draw_location[i][0]==r_pos[0] and draw_location[i][1]==r_pos[1]):
                    x = 12
                    chck = False
                    
            if(chck==True):
                blck_type = []
                for i in range(30):
                    blck_type.append(b_type)
                
                repeated_element = find_repeated_element(blck_type)
                print(repeated_element)
                blank_img = draw_block(blank_img,dir,r_pos,repeated_element)
                draw_location.append(r_pos)
                    
            
        
        #blank_img = draw_U(blank_img,dir,r_pos,3)
        final_im = blank_img+img2
        cv2.imshow('img',final_im)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
          break
    

   

if __name__ == '__main__':

    try:
        main();
    except:
        rospy.logerr('Subscriber is error');
        pass;