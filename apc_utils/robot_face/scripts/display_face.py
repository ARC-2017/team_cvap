#!/usr/bin/env python
"""
This script allows to:
- display faces on he Baxter head display
- set eyes positions (-1,+1)
- show expressions

Sergio Caccamo (KTH)
"""
__author__ =  'Sergio Caccamo <caccamo at kth.se>'
__version__=  '0.2'
__license__ = 'BSD'

# Python libs
import sys, time
import os
import datetime
import argparse
 
import rospy
import rospkg
import roslib

# numpy and scipy

import numpy as np
import math as math
from math import pi

import cv2
import cv_bridge
 
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float32MultiArray



class robot_face:

    #Frame sizes for baxter robot display    
    frame_x = 1024
    frame_y = 600
    
    center_eye_left_x = 0 + 100 + int(230/2) - int(130/2)
    center_eye_left_y = 240
    offset_left = [0,0]
    center_eye_right_x= frame_x -100 - int(230/2) - int(130/2)
    center_eye_right_y = 240
    offset_right = [0,0]
    
    #identifies Baxter' mood
    status = 0
    status_bkg = 0
    
    now = datetime.datetime.now()
    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()

    #duration of every animation
    duration = 1

    # get the file path for rospy_tutorials
    path_to_pkg = rospack.get_path('robot_face') +"/container/"  

  
    def OverlayImage(self, l_img, s_img, x_offset, y_offset):
        for c in range(0,3):
            l_img[y_offset:y_offset+s_img.shape[0], x_offset:x_offset+s_img.shape[1], c] = s_img[:,:,c] * (s_img[:,:,3]/255.0) +  l_img[y_offset:y_offset+s_img.shape[0], x_offset:x_offset+s_img.shape[1], c] * (1.0 - s_img[:,:,3]/255.0)    
        return l_img



    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        #init images
        self.init_parameter()
        self.function_init()
        # topic where to publish
        self.image_pub = rospy.Publisher('/robot/xdisplay', Image, queue_size = 1,latch=True)
        # subscribed Topic
        self.subscriber_face = rospy.Subscriber("/robot/expressions/selector", Float32, self.callback_face,  queue_size = 1)
        self.subscriber_bkg = rospy.Subscriber("/robot/background/selector", Float32, self.callback_bkg,  queue_size = 1)
        self.subscriber_eye = rospy.Subscriber("/robot/expressions/pos_eye", Float32MultiArray, self.callback_move_eye,  queue_size = 1)
        #self.subscriber_face = rospy.Subscriber("/robot/expressions/right_eye", Float32MultiArray, self.callback_moverighteye,  queue_size = 1)
        self.subscriber_duration = rospy.Subscriber("/robot/expressions/duration", Float32, self.callback_duration,  queue_size = 1)

        
    def init_parameter(self):
        self.path_background = rospy.get_param('~path_background', self.path_to_pkg+"background.png")
        self.path_background_2 = rospy.get_param('~path_background_2', self.path_to_pkg+"background_2.png")
        self.path_background_3 = rospy.get_param('~path_background_3', self.path_to_pkg+"background_3.png")
        self.path_background_4 = rospy.get_param('~path_background_4', self.path_to_pkg+"background_4.png")
        self.path_background_5 = rospy.get_param('~path_background_5', self.path_to_pkg+"background_5.png")
        self.path_eye = rospy.get_param('~path_eye', self.path_to_pkg+"eye.png")
        self.path_eye_scary = rospy.get_param('~path_eye_scary', self.path_to_pkg+"eye_scary.png")
        self.path_eye_sweet = rospy.get_param('~path_eye_sweet', self.path_to_pkg+"eye_sweet.png")
        self.path_happy = rospy.get_param('~path_happy', self.path_to_pkg+"happy.png")
        self.path_sad = rospy.get_param('~path_sad', self.path_to_pkg+"sad.png")
        self.path_crazy = rospy.get_param('~path_crazy', self.path_to_pkg+"crazy.png")
        self.path_query = rospy.get_param('~path_query', self.path_to_pkg+"query.png")
        self.path_sleepy = rospy.get_param('~path_sleepy', self.path_to_pkg+"sleepy.png")
        self.path_sweet = rospy.get_param('~path_sweet', self.path_to_pkg+"sweet.png")
        self.path_focused = rospy.get_param('~path_focused', self.path_to_pkg+"focused.png")
        self.path_sweat = rospy.get_param('~path_sweat', self.path_to_pkg+"sweat.png")


    def function_init (self):
        '''Function init , we load face expression components into img containers '''
        #Faces elements
        self.status_bkg == 0
        self.status == 0

        self.img_background = cv2.imread(self.path_background)
        self.img_background_2 = cv2.imread(self.path_background_2)
        self.img_background_3 = cv2.imread(self.path_background_3)
        self.img_background_4 = cv2.imread(self.path_background_4)
        self.img_background_5 = cv2.imread(self.path_background_5)
        self.img_eye = cv2.imread(self.path_eye,-1)
        self.img_eye_scary = cv2.imread(self.path_eye_scary,-1)
        self.img_eye_sweet = cv2.imread(self.path_eye_sweet,-1)
        self.img_happy = cv2.imread(self.path_happy,-1)
        self.img_sad = cv2.imread(self.path_sad,-1)
        self.img_crazy = cv2.imread(self.path_crazy,-1)
        self.img_query = cv2.imread(self.path_query,-1)
        self.img_sleepy = cv2.imread(self.path_sleepy,-1)
        self.img_sweet = cv2.imread(self.path_sweet,-1)
        self.img_focused = cv2.imread(self.path_focused,-1)
        self.img_sweat = cv2.imread(self.path_sweat,-1)


    def callback_duration(self, ros_data):
        '''Callback function of subscribed topic. 
        Here we set duration of the animation (not used in version 0.1 '''
        if ros_data.data > 1:
			self.duration = ros_data.data
			
    def callback_move_eye(self, ros_data):
        '''Callback function of subscribed topic. 
        Here we set duration of the animation (not used in version 0.1 '''
        if ros_data.data[0] > -1 and ros_data.data[0] < 1 and ros_data.data[1] > -1 and ros_data.data[1] < 1:
            #print "Baxter is looking around!"
            self.offset_left[0] = ros_data.data[0] * 40
            self.offset_left[1] = ros_data.data[1] * 40
            self.offset_right[0] = ros_data.data[0] * 40
            self.offset_right[1] = ros_data.data[1] * 40
            
            self.send_image_to_display()
       
       
       
        
    def callback_face(self, ros_data):
		self.status = ros_data.data
		self.send_image_to_display()
		
    def callback_bkg(self, ros_data):
		self.status_bkg = ros_data.data
		self.send_image_to_display()
		    			
    def send_image_to_display(self):
        '''Callback function of subscribed topic. 
        Here we set the Baxter' face expression '''
        img = 0

        if (self.status_bkg == 1):
            img = (self.img_background_2).copy()
        elif (self.status_bkg == 2):
            img = (self.img_background_3).copy()
        elif (self.status_bkg == 3):
            img = (self.img_background_4).copy() 
        elif (self.status_bkg == 4):
            img = (self.img_background_5).copy() 
        else:
            img = (self.img_background).copy()          
        
        if self.status == 0: #happy
            #print "Baxter is happy!"
            img_2 = self.img_happy
            img_3 = self.img_eye
            img = self.OverlayImage(img,img_3,self.center_eye_left_x + self.offset_left[0],self.center_eye_left_y +self.offset_left[1]) #left eye
            img = self.OverlayImage(img,img_3,self.center_eye_right_x + self.offset_right[0],self.center_eye_right_y +self.offset_right[1]) #right eye
            img = self.OverlayImage(img,img_2,0,0)
        elif self.status == 1: #sad
            #print "Baxter is sad!"
            img_2 = self.img_sad
            img_3 = self.img_eye
            img = self.OverlayImage(img,img_3,self.center_eye_left_x + self.offset_left[0],self.center_eye_left_y +self.offset_left[1]) #left eye
            img = self.OverlayImage(img,img_3,self.center_eye_right_x + self.offset_right[0],self.center_eye_right_y +self.offset_right[1]) #right eye
            img = self.OverlayImage(img,img_2,0,0)
        elif self.status == 2: #sweet
            #print "Baxter is sweet!"
            img_2 = self.img_sweet
            img_3 = self.img_eye_sweet   
            img = self.OverlayImage(img,img_2,0,0)
            img = self.OverlayImage(img,img_3,self.center_eye_left_x + self.offset_left[0],self.center_eye_left_y +self.offset_left[1]) #left eye
            img = self.OverlayImage(img,img_3,self.center_eye_right_x + self.offset_right[0],self.center_eye_right_y +self.offset_right[1]) #right eye
        elif self.status == 3: #crazy
            #print "Baxter is crazy!"
            img_2 = self.img_crazy
            img_3 = self.img_eye
            img = self.OverlayImage(img,img_3,self.center_eye_left_x + self.offset_left[0],self.center_eye_left_y +self.offset_left[1]) #left eye
            img = self.OverlayImage(img,img_3,self.center_eye_right_x + self.offset_right[0],self.center_eye_right_y +self.offset_right[1]) #right eye
            img = self.OverlayImage(img,img_2,0,0)
        elif self.status == 4: #sleepy
            #print "Baxter is sleeping!"
            img_2 = self.img_sleepy
            img_3 = self.img_eye
            img = self.OverlayImage(img,img_3,self.center_eye_left_x + self.offset_left[0],self.center_eye_left_y +self.offset_left[1]) #left eye
            img = self.OverlayImage(img,img_3,self.center_eye_right_x + self.offset_right[0],self.center_eye_right_y +self.offset_right[1]) #right eye
            img = self.OverlayImage(img,img_2,0,0) 
        elif self.status == 5: #focused
            #print "Baxter is focused!"
            img_2 = self.img_focused
            img_3 = self.img_eye
            img = self.OverlayImage(img,img_3,self.center_eye_left_x + self.offset_left[0],self.center_eye_left_y +self.offset_left[1]) #left eye
            img = self.OverlayImage(img,img_3,self.center_eye_right_x + self.offset_right[0],self.center_eye_right_y +self.offset_right[1]) #right eye
            img = self.OverlayImage(img,img_2,0,0)
        elif self.status == 6: #query
            #print "Baxter is doubtful!"
            img_2 = self.img_query
            img_3 = self.img_eye
            img = self.OverlayImage(img,img_3,self.center_eye_left_x + self.offset_left[0],self.center_eye_left_y +self.offset_left[1]) #left eye
            img = self.OverlayImage(img,img_3,self.center_eye_right_x + self.offset_right[0],self.center_eye_right_y +self.offset_right[1]) #right eye
            img = self.OverlayImage(img,img_2,0,0)
        elif self.status == 7: #sweaty
            #print "Baxter is sweaty!"
            img_2 = self.img_sweat
            img_3 = self.img_eye
            img = self.OverlayImage(img,img_3,self.center_eye_left_x + self.offset_left[0],self.center_eye_left_y +self.offset_left[1]) #left eye
            img = self.OverlayImage(img,img_3,self.center_eye_right_x + self.offset_right[0],self.center_eye_right_y +self.offset_right[1]) #right eye
            img = self.OverlayImage(img,img_2,0,0)
        else:
            #print "Baxter is happy!"
            img_2 = self.img_happy
            img_3 = self.img_eye
            img = self.OverlayImage(img,img_3,self.center_eye_left_x + self.offset_left[0],self.center_eye_left_y +self.offset_left[1]) #left eye
            img = self.OverlayImage(img,img_3,self.center_eye_right_x + self.offset_right[0],self.center_eye_right_y +self.offset_right[1]) #right eye
            img = self.OverlayImage(img,img_2,0,0)
                  
        self.img_buffer = img    
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(self.img_buffer, encoding="bgr8")
        self.image_pub.publish(msg)
        # Sleep to allow for image to be published.
        rospy.sleep(self.duration)       



    
def main(args):
    """RSDK Xdisplay Example: Image Display
 
   Displays a given image file on Baxter's face.
 
   Pass the relative or absolute file path to an image file on your
   computer, and the example will read and convert the image using
   cv_bridge, sending it to the screen as a standard ROS Image Message.   """
    epilog = """
   Notes:
   Max screen resolution is 1024x600.
   Images are always aligned to the top-left corner.
   Image formats are those supported by OpenCv - LoadImage().
   """
    
    rospy.init_node('display_face_robot_node', anonymous=True)
    # start robot face class
    
    rf = robot_face()

    
    try:
        rospy.spin()

    except KeyboardInterrupt:
        print "Shutting down ROS Image robot face detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)  
   
 

