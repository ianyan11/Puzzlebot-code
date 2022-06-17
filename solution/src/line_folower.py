#!/usr/bin/env python3

'''This code will recive the image from the camera from the topic /video_source/raw and will return 
a topic /line_angle of type float32. The code will follow a black line in the image utilizing a gaussian blur, and dilate and erode filters.'''
import enum
import rospy
import cv2
import numpy as np
from notCvBridge import cv2_to_imgmsg, imgmsg_to_cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from imutils.perspective import four_point_transform

class Line_Follower():
    def __init__(self) -> None:
        #Suscribes
        rospy.Subscriber('/video_source/raw', Image, self.img_callback)
        rospy.Subscriber('/ligth_state', String, self.ligth_state)
        rospy.Subscriber('/IA/signal_detection', String, self.signal_detection)

        self.tape_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.state = "Follow"
        self.ligth_state = " "
        self.iter = 0
        self.begin = rospy.get_time()
        self.signal = ""

    def signal_detection(self, str):
        self.signal=str.data

    
    def ligth_state (self, msg):
        """Method which recieves semaphore state"""
        self.ligth_state = msg.data
    

    def filter_img (self, img):
        """Method to apply image processing"""
        #gaussian blur
        img = cv2.GaussianBlur(img, (3,3), 0)
        #Dilate and Erode
        img = cv2.dilate(img, None, iterations=3)
        img = cv2.erode(img, None, iterations=3)
        return img

    def state_machine(self, half, min_index):
        if (self.state == "Follow"):
            self.twist.angular.z = -(min_index-half)/half *.3
            self.twist.linear.x = 0.22 #-abs(t.angular.z)*(.3/.5)


        
        elif (self.state == "Intersection"):
            self.twist.angular.z = 0.0
            self.twist.linear.x = 0.0
                
        
        elif (self.state == "Red"):
            self.twist.angular.z = 0.0
            self.twist.linear.x = 0.0



        elif (self.state == "Green" and self.signal == "Ahead Only" """self.iter==0"""):
            self.twist.angular.z = 0.0
            self.twist.linear.x = 0.2

        elif(self.state == "Green" and self.signal == "Turn Right" """self.iter==1"""):
            #print("alla")
            self.twist.angular.z =-0.1
            self.twist.linear.x = 0.15
        
        self.tape_pub.publish(self.twist)

                
        
    def img_callback(self, msg):
        #convert to opencv format
        image = imgmsg_to_cv2(msg)
        #rotate image and convert to hsv
        image = cv2.rotate(image, cv2.ROTATE_180) 
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 
        #reduce image size
        img = cv2.resize(hsv, (int(hsv.shape[1] * .2),int(hsv.shape[0] * .25)))
        #Crop one image for intersection and other for line follower
        imgLine = four_point_transform(img, np.array([[60,140],[len(img[0])-60,140],[len(img[0]),len(img)],[0, len(img)]]))
        imgInter = img[int(len(img)*.6):int(len(img)*0.9),30:len(img[0])-30]
        #Apply image processing
        imgLine = self.filter_img(imgLine)
        imgInter = self.filter_img(imgInter)
        #Treshing image to help the edge recognition
        treshedInter = cv2.threshold(imgInter, 138, 255, cv2.THRESH_BINARY)[1]
        v = np.sum(imgLine, axis=0)
        h = np.sum(treshedInter, axis=1)
        hMean = np.mean(treshedInter, axis=1) #Compute means to have a treshold
        #get index of smallest value
        min_indexV = np.argmin(v)
        min_indexH = np.argmin(h)
        print(self.state)
        #rospy.loginfo(self.state)
        if ((min_indexH < 60)):
            if (self.state == "Follow"): 
                if(hMean[min_indexH] < 145):
                    self.state = "Intersection"             
            elif (self.state == "Intersection"):
                if (self.ligth_state == "RED"):
                    self.state = "Red"
                elif (self.ligth_state == "GREEN"):
                    self.begin = rospy.get_time()
                    self.state = "Green"
            elif (self.state == "Red"):
                if (self.ligth_state == "GREEN"):
                    self.begin = rospy.get_time()
                    self.state = "Green"
            elif(self.state == "Green"):
                print(rospy.get_time()-self.begin)
                if(rospy.get_time()-self.begin>2):
                    self.state="Follow"
                    if(self.iter==0):
                        self.iter=1
                    else:
                        self.iter=0
        #else:
            #self.state = "Follow" 
        
        half = int(len(v)/2)+1
        self.state_machine(half, min_indexV)

def main():
    rospy.init_node('linefollower', anonymous=True)
    h = Line_Follower()
    rospy.spin()

if __name__ == "__main__":
    main()
    
