#!/usr/bin/env python
from math import atan2
from re import X
import rospy
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
from geometry_msgs.msg import Pose, Point, Twist, Vector3, Quaternion, TransformStamped
from tf.transformations import euler_from_quaternion

#Clase control que ejecuta las acciones de contol y contiene las variables involucradas
class Control:
    #Init pose object
    pose = Pose()
    #Topic we are going to publish 
    posPub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    #The message is going to be a Twist class objects
    msg = Twist()
    #Sintonization variables
    kpl = 0.8
    kpt = 0.5

    def __init__(self):
        self.tf_buffer = Buffer()
        #Start tf listener
        self.listener = TransformListener(self.tf_buffer)

    #Method to set the position
    def setDesiredPosition(self, data):
        #Send the position using the broadcast method
        self.broadcastTransform(data)
        x=True
        while(x):
            try:
                #Keep broadcasting the data
                self.broadcastTransform(data)
                #Update x with the return value of control method wich is
                #   True if we finished the control or false if not
                x = (not self.startControl())
            except:
                None
        #Set velocities to 0
        self.setVelocities(Twist())

    #Set the pose
    def setPosition(self, data):
        self.pose = data

    #Method which execute control
    def startControl(self):
        #Look for the transform between robot and points
        transformObject = self.tf_buffer.lookup_transform("robot", "point", rospy.Time())
        rospy.loginfo(transformObject)
        #Extract translation
        trans = transformObject.transform.translation
        #Compute PID
        self.pidCalculation(trans)
        #Return true if robot reached the desired position
        return(abs(trans.x)<0.01 and abs(trans.y)<0.01)
        
    #Compute PID method
    def pidCalculation(self, error):
        #Compute and Set linear and angular speed on msg
        self.msg.linear.x = self.kpl*error.x
        yaw = atan2(error.y,error.x)
        self.msg.angular.z = self.kpt*yaw
        #Publish velocities
        self.setVelocities(self.msg)

        
    #Publish the message for control propourses
    def setVelocities(self, speed):
        self.posPub.publish(speed)
    
    def broadcastTransform(self, data):
        #Init transform broadcaster
        br = TransformBroadcaster()
        t = TransformStamped()
        #Fill the transform with time, frames, translation and orientation data
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "point"
        t.transform.translation = Vector3(data.x, data.y, data.z) #Translation
        t.transform.rotation = Quaternion(0,0,0,1) #Orientation
        br.sendTransform(t)
		





def main():
    rospy.init_node('Control', anonymous=True)
    
    control = Control()
    rospy.Subscriber("DesiredPosition",Point, control.setDesiredPosition)
    rospy.Subscriber("Position",Pose, control.setPosition)
    rospy.spin()
        
if __name__ == "__main__":
    main()
