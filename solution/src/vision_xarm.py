#!/usr/bin/env python
#from _future_ import division

from re import I
import rospy
import cv2 as cv
import numpy as np
import sys
import cv_bridge 
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist,Vector3 



class Image_cv:
    image_pub = rospy.Publisher("/video_source/view",Image, queue_size=10)
    light_state = rospy.Publisher('/light_state', String, queue_size = 10)
    posPub = rospy.Publisher('/marker_properties', Vector3, queue_size = 10)

    icolGreen = (50,75,75,70,255,255)    # Green
    icolRed1 = (0,100,20,8,255,255)      # Red
    icolRed2 = (160,100,20,185,255,255)  # Red
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

        #self.light_state.publish(String("none"))

        #calculamos limites del filtro
        colorLowR1 = np.array([self.icolRed1[0],self.icolRed1[1],self.icolRed1[2]])
        colorHighR1 = np.array([self.icolRed1[3],self.icolRed1[4],self.icolRed1[5]])
        colorLowR2 = np.array([self.icolRed2[0],self.icolRed2[1],self.icolRed2[2]])
        colorHighR2 = np.array([self.icolRed2[3],self.icolRed2[4],self.icolRed2[5]])
        
        #procesamos Rojos
        maskR1 = cv.inRange(cv_image, colorLowR1, colorHighR1)
        maskR2 = cv.inRange(cv_image, colorLowR2, colorHighR2)
        maskR = cv.add(maskR1,maskR2)
        resultR=cv.bitwise_and(cv_image, cv_image, mask = maskR) 
        #resizeR=cv.resize(resultR,(255,144))
        blurR=cv.medianBlur(resultR,3)
        bgrR=cv.cvtColor(blurR,cv.COLOR_HSV2BGR)
        grayR=cv.cvtColor(bgrR,cv.COLOR_BGR2GRAY)
        numCirclesR=cv.HoughCircles(grayR,cv.HOUGH_GRADIENT,1.2,700,param1=35,param2=10,minRadius=130,maxRadius=150) #se puede mover el maxRadius
            

        if numCirclesR is not None:
            numCirclesR=np.uint16(np.around(numCirclesR))
            self.light_state.publish(String("red_light"))
            print("red circle was found")
            for j in numCirclesR[0,:]:
                xR=j[0] -690 #x
                yR=j[1] -360#y
                rR=j[2] #radio
                posR=Vector3(xR,yR,rR)
                
                self.posPub.publish(posR)
        else:
            self.light_state.publish(String("none"))
      

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
