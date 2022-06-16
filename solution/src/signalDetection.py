#!/usr/bin/env python3

"Find Circles in an image"
from std_msgs.msg import String
import sensor_msgs.msg
import numpy as np
import cv2
import tensorflow as tf
from PIL import Image, ImageOps
import rospy
import rospkg
from notCvBridge import cv2_to_imgmsg, imgmsg_to_cv2
from tensorflow.python.framework import graph_io

class SignalDetection():
    def __init__(self):
        rospack = rospkg.RosPack()
        tf.ke
        self.model = load_model(rospack.get_path('solution')+('/src/keras_model.h5'))
        rospy.Subscriber('/video_source/raw', sensor_msgs.msg.Image, self.signal_detection)
        self.detectionPublisher = rospy.Publisher('/signal_detection', String, queue_size=10)
        
    def signal_detection(self, msg):
        #convert to opencv format
        image = imgmsg_to_cv2(msg)
        image = cv2.resize(image,(0,0),fx=0.3,fy=0.3)
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Blur image
        blur = cv2.medianBlur(gray, 3)
        # Apply edge detection
        edges = cv2.Canny(blur, 300, 300, apertureSize=3)
        cv2.imshow("edges", edges)
        cv2.waitKey(1)
        # Find circles
        circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, .8, 50, param1=130, param2=30, minRadius=1, maxRadius=100)
        # Draw circles
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                try:
                    crop_img = image[i[1]-i[2]:i[1]+i[2], i[0]-i[2]:i[0]+i[2]]
                    pre = self.image_classification(crop_img)
                    self.detectionPublisher.publish(pre)
                except:
                    None

    def image_classification(self, image):
        size = (224, 224)
        data = np.ndarray(shape=(1, 224, 224, 3), dtype=np.float32)
        img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        img =Image.fromarray(img)
        pil = ImageOps.fit(img, size, Image.ANTIALIAS)
        # run the inference
        #turn the image into a numpy array
        image_array = np.asarray(pil)
        # Normalize the image
        normalized_image_array = (image_array.astype(np.float32) / 127.0) - 1
        # Load the image into the array
        data[0] = normalized_image_array
        prediction = self.model.predict(data)
        max = prediction.argmax()
        if(max==0):
            return("Turn Right")
        elif(max==1):
            return("Stop")
        elif(max==2):
            return("No Limit")
        elif(max==3):
            return("Ahead Only")
                    


def main():
    rospy.init_node('signal_detection', anonymous=True)
    signal_detection = SignalDetection()
    rospy.spin()

if __name__ == '__main__':
    main()

    
    


    
    
