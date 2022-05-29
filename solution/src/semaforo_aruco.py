#!/usr/bin/env python3
# # import the necessary packages
import rospy
import cv2 as cv
import cv_bridge 
import numpy as np
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from sensor_msgs.msg import Image


image_pub = rospy.Publisher("/video_source/view",Image, queue_size=10)
light_state = rospy.Publisher('/light_state', String, queue_size = 5)
posPub = rospy.Publisher('/marker_properties', Vector3, queue_size = 5)


icolGreen = (50,75,75,70,255,255)    # Green
icolRed1 = (0,100,20,8,255,255)      # Red
icolRed2 = (175,100,20,179,255,255)  # Red
bridge = cv_bridge.CvBridge()
frame = None

def semaforo_detect(data):
    global frame
    try:
        #pasa de imagen a formato cv2
        frame = bridge.imgmsg_to_cv2(data, 'bgr8')
        process_frame()
    except cv_bridge.CvBridgeError as e:
        print(e)


def process_frame():
    global frame
    #calculamos limites del filtro

    colorLowR1 = np.array([icolRed1[0],icolRed1[1],icolRed1[2]])
    colorHighR1 = np.array([icolRed1[3],icolRed1[4],icolRed1[5]])
    colorLowR2 = np.array([icolRed2[0],icolRed2[1],icolRed2[2]])
    colorHighR2 = np.array([icolRed2[3],icolRed2[4],icolRed2[5]])
    colorLowG = np.array([icolGreen[0],icolGreen[1],icolGreen[2]])
    colorHighG = np.array([icolGreen[3],icolGreen[4],icolGreen[5]])

       
    hsv=cv.cvtColor(frame,cv.COLOR_BGR2HSV)

    """
    maskG = cv.inRange(hsv, colorLowG, colorHighG)
    resultG=cv.bitwise_and(frame, frame, mask = maskG) 
    #resizeG=cv.resize(resultG,(255,144))
    blurG=cv.medianBlur(resultG,3)
    bgrG=cv.cvtColor(blurG,cv.COLOR_HSV2BGR)
    grayG=cv.cvtColor(bgrG,cv.COLOR_BGR2GRAY)
    numCirclesG=cv.HoughCircles(grayG,cv.HOUGH_GRADIENT,1.2,50,param1=40,param2=20,minRadius=80,maxRadius=200) #se puede mover el maxRadius
    """
    #procesamos Rojos
    maskR1 = cv.inRange(hsv, colorLowR1, colorHighR1)
    maskR2 = cv.inRange(hsv, colorLowR2, colorHighR2)
    maskR = cv.add(maskR1,maskR2)
    resultR=cv.bitwise_and(frame, frame, mask = maskR) 
    #resizeR=cv.resize(resultR,(255,144))
    blurR=cv.medianBlur(resultR,3)
    bgrR=cv.cvtColor(blurR,cv.COLOR_HSV2BGR)
    grayR=cv.cvtColor(bgrR,cv.COLOR_BGR2GRAY)

    numCirclesR=cv.HoughCircles(grayR,cv.HOUGH_GRADIENT,1,50,param1=30,param2=10,minRadius=80,maxRadius=200) #se puede mover el maxRadius
    

    if numCirclesR is not None:
        numCirclesR=np.uint16(np.around(numCirclesR))
        light_state.publish(String("red_light"))
        print("red circle was found")
        for j in numCirclesR[0,:]:
            xR=j[0] -690 #x
            yR=j[1] -360#y
            rR=j[2] #radio
            posR=Vector3(xR,yR,rR)
                
            posPub.publish(posR)
    else:
        light_state.publish(String("none"))
    """
    if numCirclesG is not None:
        numCirclesG=np.uint16(np.around(numCirclesG))
        #self.light_state.publish(String("green_light"))
        print("green circle was found")
        for i in numCirclesG[0,:]:
            xG=i[0] #x
            yG=i[1] #y
            rG=i[2] #radio
            posG=Vector3(xG,yG,rG)

            posPub.publish(posG)
    """

def main():
    #Init the of odometry
    rospy.init_node('image_converter', anonymous=True)
    rospy.Subscriber("/video_source/raw", Image,semaforo_detect)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()
