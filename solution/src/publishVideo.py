import jetson.utils
import argparse
import sys
import rospy
import cv_bridge 
from sensor_msgs.msg import Image

# parse command line

image_pub = rospy.Publisher("/video_source/raw", Image, queue_size=10)
# create video sources & outputs
input = jetson.utils.videoSource(argv=sys.argv)
r = rospy.Rate(100)

# capture frames until user exits
while not rospy.is_shutdown():
    image = input.Capture(format='rgb8')  # can also be format='rgba8', 'rgb32f', 'rgba32f'
    image_pub.publish(image)
    r.sleep()
