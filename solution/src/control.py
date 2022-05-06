#!/usr/bin/env python2
from math import atan2
from os import kill
from re import X
import rospy
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
from geometry_msgs.msg import Pose, Point, Twist, Vector3, Quaternion, TransformStamped
from control_msgs.msg import PidState
from tf.transformations import euler_from_quaternion
from simple_pid import PID

#Clase control que ejecuta las acciones de contol y contiene las variables involucradas
class Control:
    #Init pose object
    pose = Pose()
    #Topic we are going to publish 
    posPub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    #The message is going to be a Twist class objects
    msg = Twist()

    dt = 0.0108
    #Sintonization variables
    #linear PID
    pid_l = PID(0.3, 0.001, 0.02, setpoint = 0, sample_time=dt) 
    #linearPub = rospy.Publisher('linearPID', PidState, queue_size = 10)
    #linear = PidState()
    #angular PID
    pid_a = PID(2, 0.02, 0.02, setpoint = 0, sample_time=dt)
    #angularPub = rospy.Publisher('angularPID', PidState, queue_size = 10)
    #angular = PidState()

    def __init__(self):
        self.tf_buffer = Buffer()
        #Start tf listener
        self.listener = TransformListener(self.tf_buffer)
        #self.linear.p_term = self.pid_l.Kp
        #self.linear.i_term = self.pid_l.Ki
        #self.linear.d_term = self.pid_l.Kd
        #self.angular.p_term = self.pid_a.Kp
        #self.angular.i_term = self.pid_a.Ki
        #self.angular.d_term = self.pid_a.Kd




    #Method to set the position
    def setDesiredPosition(self, data):
        #Send the position using the broadcast method
        self.broadcastTransform(data)
	#r = rospy.Rate(60)
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
	    #r.sleep()
        #Set velocities to 0
        self.setVelocities(Twist())

    #Set the pose
    def setPosition(self, data):
        self.pose = data

    #Method which execute control
    def startControl(self):
        #Look for the transform between robot and points
        transformObject = self.tf_buffer.lookup_transform("robot", "point", rospy.Time())
        #Extract translation
        trans = transformObject.transform.translation
        #Compute PID
        self.pidCalculation(trans)

        #Return true if robot reached the desired position
        return(abs(trans.x)<0.01 and abs(trans.y)<0.01)
        
    #Compute PID method
    def pidCalculation(self, error):
        #Compute and Set linear and angular speed on msg
        self.msg.linear.x = -self.pid_l(error.x)
        #self.linear.header.stamp = rospy.Time.now()
        #self.linear.header.frame_id = "robot"
        #linearComp = self.pid_l.components
        #self.linear.error = error.x
        #self.linear.p_error = linearComp[0]
        #self.linear.i_error = linearComp[1]
        #self.linear.d_error = linearComp[2]
        #self.linear.output = self.msg.linear.x
        #self.linearPub.publish(self.linear)

        yaw = atan2(error.y,error.x)
        self.msg.angular.z = -self.pid_a(yaw)
        #self.angular.header.stamp = rospy.Time.now()
        #self.angular.header.frame_id = "robot"
        #angularComp = self.pid_a.components
        #self.angular.error = yaw
        #self.angular.p_error = angularComp[0]
        #self.angular.i_error = angularComp[1]
        #self.angular.d_error = angularComp[2]
        #self.angular.output = self.msg.angular.z
        #self.angularPub.publish(self.angular)



        #rospy.loginfo("pid_l: ", linearComp)
        #rospy.loginfo("pid_t:", angularComp)
        #Publish velocities
        self.setVelocities(self.msg)
        #self.posPub.publish(self.msg)

        
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
