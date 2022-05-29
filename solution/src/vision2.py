#!/usr/bin/env python
#from _future_ import division

import rospy
import cv2 as cv
import numpy as np
import sys
import cv_bridge 
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class Image_cv:
    image_pub = rospy.Publisher("/video_source/view",Image, queue_size=10)
    light_state = rospy.Publisher('/light_state', String, queue_size = 10)

    icolGreen = (50,75,75,70,255,255)    # Green
    icolRed1 = (0,100,20,8,255,255)      # Red
    icolRed2 = (175,100,20,179,255,255)  # Red
    icolBlue = (100,80,50,160,255,255)  # Blue
    bridge = cv_bridge.CvBridge()    

    def _init_(self):
       None
    
    def callback(self,data):
        try:
            #pasa de imagen a formato cv2
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except cv_bridge.CvBridgeError as e:
            print(e)

        self.light_state.publish(String("none"))

        #calculamos limites del filtro
        colorLowR1 = np.array([self.icolRed1[0],self.icolRed1[1],self.icolRed1[2]])
        colorHighR1 = np.array([self.icolRed1[3],self.icolRed1[4],self.icolRed1[5]])
        colorLowR2 = np.array([self.icolRed2[0],self.icolRed2[1],self.icolRed2[2]])
        colorHighR2 = np.array([self.icolRed2[3],self.icolRed2[4],self.icolRed2[5]])
        colorLowG = np.array([self.icolGreen[0],self.icolGreen[1],self.icolGreen[2]])
        colorHighG = np.array([self.icolGreen[3],self.icolGreen[4],self.icolGreen[5]])
        colorLowB = np.array([self.icolBlue[0],self.icolBlue[1],self.icolBlue[2]])
        colorHighB = np.array([self.icolBlue[3],self.icolBlue[4],self.icolBlue[5]])
        #procesamos Verdes
        hsv=cv.cvtColor(cv_image,cv.COLOR_BGR2HSV)
        maskG = cv.inRange(hsv, colorLowG, colorHighG)
        resultG=cv.bitwise_and(cv_image, cv_image, mask = maskG) 
        #resizeG=cv.resize(resultG,(255,144))
        blurG=cv.medianBlur(resultG,3)
        bgrG=cv.cvtColor(blurG,cv.COLOR_HSV2BGR)
        grayG=cv.cvtColor(bgrG,cv.COLOR_BGR2GRAY)
        numCirclesG=cv.HoughCircles(grayG,cv.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=10,maxRadius=70) #se puede mover el maxRadius

        #procesamos Rojos
        maskR1 = cv.inRange(hsv, colorLowR1, colorHighR1)
        maskR2 = cv.inRange(hsv, colorLowR2, colorHighR2)
        maskR = cv.add(maskR1,maskR2)
        resultR=cv.bitwise_and(cv_image, cv_image, mask = maskR) 
        #resizeR=cv.resize(resultR,(255,144))
        blurR=cv.medianBlur(resultR,3)
        bgrR=cv.cvtColor(blurR,cv.COLOR_HSV2BGR)
        grayR=cv.cvtColor(bgrR,cv.COLOR_BGR2GRAY)
        numCirclesR=cv.HoughCircles(grayR,cv.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=10,maxRadius=70) #se puede mover el maxRadius
   
        #procesamos Azules
        hsv=cv.cvtColor(cv_image,cv.COLOR_BGR2HSV)
        maskB = cv.inRange(hsv, colorLowB, colorHighB)
        resultB=cv.bitwise_and(cv_image, cv_image, mask = maskB) 
        #resizeR=cv.resize(resultR,(255,144))
        blurB=cv.medianBlur(resultB,3)
        bgrB=cv.cvtColor(blurB,cv.COLOR_HSV2BGR)
        grayB=cv.cvtColor(bgrB,cv.COLOR_BGR2GRAY)
        numCirclesB=cv.HoughCircles(grayB,cv.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=10,maxRadius=70) #se puede mover el maxRadius
        #subimos rojo procesado
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(grayB, encoding="passthrough"))


        if numCirclesG is not None:
            self.light_state.publish(String("green_light"))
            print("green circle was found")


        if numCirclesR is not None:
            self.light_state.publish(String("red_light"))
            print("red circle was found")
            
        if numCirclesB is not None:
            self.light_state.publish(String("blue_light"))
            print("blue circle was found")

      

def main():
    rospy.init_node('image_converter', anonymous=True)
    ic = Image_cv()
    rospy.Subscriber("/video_source/raw", Image, ic.callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()
