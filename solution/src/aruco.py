#!/usr/bin/env python3
# # import the necessary packages
import rospy
import cv2 
import cv_bridge 
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image


#image_pub = rospy.Publisher("/video_source/aruco", Image, queue_size=10)
aruco_pos = rospy.Publisher("/aruco/pose", Vector3, queue_size=5)

bridge = cv_bridge.CvBridge()
frame = None

def arucoDetector(data):
    global frame
    try:
        #pasa de imagen a formato cv2
        frame = bridge.imgmsg_to_cv2(data, 'bgr8')
        process_frame()
    except cv_bridge.CvBridgeError as e:
        print(e)


def process_frame():
    global frame
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    arucoParams = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame,arucoDict, parameters = arucoParams)
    
    # verify *at least* one ArUco marker was detected
    if len(corners) > 0:
        # flatten the ArUco IDs list
        ids = ids.flatten()
        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
            # extract the marker corners (which are always returned in
            # top-left, top-right, bottom-right, and bottom-left order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
                # draw the bounding box of the ArUCo detection
            cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
            # compute and draw the center (x, y)-coordinates of the ArUco
            # marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)


            a = topLeft[0] + topRight[0]
            b =bottomLeft[0] + bottomRight[0]
            aruco_pos.publish(Vector3(cX-1280/2,cY-720/2, cv2.contourArea(markerCorner)))
            #print("area = ", cv2.contourArea(markerCorner))
            # show the output image
        #image_pub.publish(bridge.cv2_to_imgmsg(frame, encoding="passthrough"))
    #else:
        #image_pub.publish(bridge.cv2_to_imgmsg(frame, encoding="passthrough"))
    else:
        aruco_pos.publish(Vector3())


def main():
    #Init the of odometry
    rospy.init_node('ArucoDetector', anonymous=True)
    rospy.Subscriber("/video_source/raw", Image, arucoDetector)
    
    rospy.spin()


if __name__ == "__main__":
    main()
