#!/usr/bin/env python
import numpy as np
import cv2
import math
import gi
import os
import rospy
from std_msgs.msg import String
import imutils
from skimage import exposure
from geometry_msgs.msg import Twist
import math

font = cv2.FONT_HERSHEY_COMPLEX

i = 0

depth = 0

turnlast = 0

# Capture Video and set resolution
pipeline_string = "udpsrc port=5200 ! application/x-rtp,encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegdec ! videoconvert ! appsink "

video_capture = cv2.VideoCapture(pipeline_string, cv2.CAP_GSTREAMER)

if __name__ == '__main__':

    rospy.init_node('Pi_car_derp', anonymous=True)

    while video_capture.isOpened():
        
        # Where the feature detection goes
        ret, img = video_capture.read()
        imCopy = img.copy()
        img = cv2.GaussianBlur(img, (5, 5), 0)
        roi = img[250:480, 0:600]

        if not ret:
            print('empty frame')

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        lower_black = np.array([0,0,0])
        upper_black = np.array([180,255,50])

        mask_black = cv2.inRange(hsv, lower_black, upper_black)
###########


        white_background = np.zeros(roi.shape, roi.dtype)
        white_background[:,:] = (0,0,0)


        output_black = cv2.bitwise_and(white_background, white_background, mask = mask_black)


##########
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        ret,thresh = cv2.threshold(gray,60,255,cv2.THRESH_BINARY_INV)

        contours,hierarchy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)

        roi_blackline = gray

        forward = 1
        turn = 0
        updown = 0
        strafe = 0

        if len(contours) > 0:

            c = max(contours, key=cv2.contourArea)

            M = cv2.moments(c)

            try:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                cv2.line(roi,(cx,0),(cx,480), (255,0,0), 3)
                cv2.line(roi,(0,cy),(600,cy), (255,0,0), 3)
                cv2.drawContours(roi, contours, -1, (0,255,255),1)

            except:
                turn = 300
                forward = 0

            print cx

            turn = cx

            turnlast = turn

            print turnlast

        else:
            turn = turnlast

        #Rcolor_filtered = np.hstack([img, output])
        cv2.imshow("Crop", mask_black)
        cv2.imshow("Normal View", img)
        cv2.imshow("AUTOROV", roi_blackline)

        if cv2.waitKey(1) & 0xFF == ord('f'):
            twist.linear.x = 0
            twist.linear.y = 0
            break

        # When everything done, release the capture
        pub = rospy.Publisher("Twist", Twist, queue_size=1)

        # Create Twist message & add linear x and angular z from left joystick
        twist = Twist()
        twist.linear.x =  forward      # Move forward backward
        twist.linear.y =  turn         # Strafe
#        twist.linear.z = updown
#        twist.angular.z = turn        # Turn left right

        # record values to log file and screen

        rospy.loginfo("Linear.x: %f :: Linear.y: %f :: Linear.z: %f :: Angular.z: %f ", twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z)

        # publish cmd_vel move command to ROV
        pub.publish(twist)


        # When everything done, release the capture
    pub = rospy.Publisher("Twist", Twist, queue_size=1)

    # Create Twist message & add linear x and angular z from left joystick
    twist = Twist()
    twist.linear.x =  0               # Move forward backward
    twist.linear.y =  0               # Strafe
#    twist.linear.z = 0
#    twist.angular.z = 0              # Turn left right

    # record values to log file and screen
    rospy.loginfo("Linear.x: %f :: Linear.z: %f :: Angular.z: %f ", twist.linear.x, twist.linear.z, twist.angular.z)

    # publish cmd_vel move command to ROV
    pub.publish(twist)


    video_capture.release()
    cv2.destroyAllWindows()
