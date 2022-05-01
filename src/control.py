#!/usr/bin/env python
from math import atan2
from re import X
import rospy
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
from geometry_msgs.msg import Pose, Point, Twist, Vector3, Quaternion, TransformStamped
from tf.transformations import euler_from_quaternion


class Control:
    pose = Pose()
    posPub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    msg = Twist()
    kpl = 0.8
    kpt = 0.5
    def __init__(self):
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer)

    def setDesiredPosition(self, data):
        self.broadcastTransform(data)
        x=True
        while(x):
            try:
                self.broadcastTransform(data)
                x = (not self.startControl())
            except:
                None
            
        self.setVelocities(Twist())


    def setPosition(self, data):
        self.pose = data


    def startControl(self):
        
        transformObject = self.tf_buffer.lookup_transform("robot", "point", rospy.Time())
        rospy.loginfo(transformObject)
        
        trans = transformObject.transform.translation
        self.pidCalculation(trans)
        return(abs(trans.x)<0.01 and abs(trans.y)<0.01)
        
        

    def pidCalculation(self, error):
        self.msg.linear.x = self.kpl*error.x

        yaw = atan2(error.y,error.x)
        self.msg.angular.z = self.kpt*yaw
        self.setVelocities(self.msg)

        

    def setVelocities(self, speed):
        self.posPub.publish(speed)
        
    def broadcastTransform(self, data):
        br = TransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "point"
        t.transform.translation = Vector3(data.x, data.y, data.z)
        t.transform.rotation = Quaternion(0,0,0,1)
        br.sendTransform(t)
		





def main():
    rospy.init_node('Control', anonymous=True)
    
    control = Control()
    rospy.Subscriber("DesiredPosition",Point, control.setDesiredPosition)
    rospy.Subscriber("Position",Pose, control.setPosition)
    rospy.spin()
        
if __name__ == "__main__":
    main()
