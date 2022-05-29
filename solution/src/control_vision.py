#!/usr/bin/env python
from math import atan2
from re import X
import rospy
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
from geometry_msgs.msg import Pose, Point, Twist, Vector3, Quaternion, TransformStamped
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion


class Control_vis:
    pose = Pose()
    #Area threshold for control propourses
    #areaCircleThreshold = 50000
    areaMarkerThreshold = 150
    #Publisher of velocities
    posPub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    msg = Twist()
    #PID constants
    kpl = 0.001
    kpt = 0.00005

    dt = 0.01087
    
    #Variable of the circle data
    circledata = Vector3()

    def __init__(self):
        """Init of the class node"""
        #Variable which keeps the color of semaphore
        
    def ligthState(self, data):
        """Method to asing the ligth state to class variable"""
        self.light_state = data
        #print(data)

    def moveToObjective (self, data):
        """Callback method after reciving information of the circle
            Determine if we can move and start the control
            Input: data: The information of the marker need it for control: Vector3"""

        if ((self.light_state.data != "red_light") and (self.light_state.data == "blue_light")):
            #If startControl return True, then we reached the objetive
            if(self.startControl(data.z, data.x)):
                #Stop the robots
                self.setVelocities(Twist())
        else:
            print("Semaforo rojo")
            self.setVelocities(Twist())

    def startControl(self, mesuredArea, centerDistance):
        """ Method to start de control operation
                Input: mesuredArea:the area mesured on the vision node : float
                        centerDistance : circle distance from center: float: pixels"""
        #Compute area error
        errorArea = self.areaMarkerThreshold - mesuredArea 
        self.pidCalculation(errorArea, centerDistance)
        #Return true if we reached the objective 
        return(abs(mesuredArea)>self.areaMarkerThreshold and abs(centerDistance)<200)
        
        

    def pidCalculation(self, errorArea, centerDistance):
        """ Method to start de control operation
            Input: errorArea:the area error previously computed: float
                centerDistance : circle distance from center: float: pixels
                        
            The method compute the PID based on area diference for linear velocity
            and the distance of circle center for the angular control
                        """   
        #First center the point we are moving to
        if (centerDistance > 200):
            #Computing angular velocitie
            print("Moviendose a los lados")
            self.msg.angular.z = self.kpt*centerDistance
            self.msg.linear.x = 0
        
        if (centerDistance < -200):
            #Computing angular velocitie
            print("Moviendose a los lados")
            self.msg.angular.z = -self.kpt*centerDistance
            self.msg.linear.x = 0

        else:
            #Compute linear velocities based on area
            print("Moviendose al frente")
            self.msg.angular.z = 0
            self.msg.linear.x = self.kpl*errorArea

        #Publish the velocities msg
        self.setVelocities(self.msg)

        
    def setVelocities(self, speed):
        """Method for publishing PID velocities"""
        self.posPub.publish(speed)

def main():
    rospy.init_node('Control', anonymous=True)
    
    control = Control_vis()
    #Subscribers
    rospy.Subscriber('/light_state', String, control.ligthState)
    rospy.Subscriber('/marker_properties', Vector3, control.moveToObjective)
    rospy.spin()
        
if __name__ == "__main__":
    main()
