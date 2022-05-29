#!/usr/bin/env python
from math import atan2, sqrt
from re import X

import rospy
from geometry_msgs.msg import Pose, Point, Twist, Vector3, Quaternion, TransformStamped
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion


class Control_vis:
    pose = Pose()
    #Limit threshold
    pixel_threshold = 20
    #Publisher of velocities
    posPub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    msg = Twist()
    #PID constants
    kpl = 0.001
    kpt = 0.03
    dt = 0.01087

    def __init__(self):
        """Init of the class node"""

    def moveToObjective (self, data):
        """ Callback method after reciving information of the marker and
            start control
            Input: data: The information of the marker need it for control: Vector3"""
            #If startControl return True, then we reached the objetive
        if(self.startControl(data.x, data.y)):
            #Stop the robots
            self.setVelocities(Twist())

    def startControl(self, xCoord, yCoord):
        """ Method to start de control operation
            Input: xCoord: x coordinates of the maker respect to img center: float
            
            yCoord : y coordinate of the maker respect to img center: float"""

        self.pidCalculation(xCoord, yCoord)
        #Return true if we reached the objective 
        return(abs(xCoord)<self.pixel_threshold and abs(yCoord)<self.pixel_threshold)
        
        

    def pidCalculation(self, xCoord, yCoord):
        """ Method to start de control operation
            Input: xCoord: x coordinates of the maker respect to img center: float
                yCoord : y coordinate of the maker respect to img center: float
                        
            The method compute the PID based on the coordinates of the marker to
            move the robot to it

            First it moves in angle and the linear. When it finishes its angular 
            displacement the center of the marker is allign with the x axis
                        """   
        #If xcoord is not allign with the x axis in this threshold
        if (abs(xCoord) > self.pixel_threshold):
            yaw = atan2(yCoord, xCoord)
            #Computing angular velocity
            self.msg.angular.z = self.kpt*yaw
            self.msg.linear.x = 0.0
        elif (abs(yCoord) > self.pixel_threshold):
            linearDis = yCoord
            self.msg.angular.z = 0.0
            #Compute linear velocity
            self.msg.linear.x = float(self.kpl*linearDis)
        else:
            self.msg.angular.z = 0.0
            self.msg.linear.x = 0.0

        #Publish the velocities msg
        self.setVelocities(self.msg)

    def setVelocities(self, speed):
        """Method for publishing PID velocities"""
        self.posPub.publish(speed)

def main():
    rospy.init_node('Control_xarm', anonymous=True)
    
    control = Control_vis()
    #Subscribers
    rospy.Subscriber('/aruco/pose', Vector3, control.moveToObjective)
    rospy.spin()
        
if __name__ == "__main__":
    main()
