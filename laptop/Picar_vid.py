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

        #Rcolor_filtered = np.hstack([img, output])
        cv2.imshow("Normal View", img)

        if cv2.waitKey(1) & 0xFF == ord('f'):
            break


    video_capture.release()
    cv2.destroyAllWindows()
