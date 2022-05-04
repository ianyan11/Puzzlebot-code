#!/usr/bin/env python

from math import cos, sin, pi

import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Float32, Empty
from geometry_msgs.msg import Pose, TransformStamped, Vector3
import numpy as np

#Class which has some characteristics of the robot and 
#	has the odometry variables
class Odom:
	#Initialize pose class
	p = Pose()
	#Speed and delta time
	wl = 0
	wr = 0
	dt = 0.01
	#posPub = rospy.Publisher('Position', Pose, queue_size = 10)
	r = 0.05 # Wheel radius, 0.05m
	l = 0.19 # Distance between robot wheels 0.19m

	#Get the speed of left wheel
	def lSpeed(self, data):
		self.wl = data.data
	#Get the speed of rigth wheel
	def rSpeed(self, data):
		self.wr = data.data
	
	#Calculate position based on the integration of velocities
	def calculate(self):
		euler = euler_from_quaternion((self.p.orientation.x,self.p.orientation.y,self.p.orientation.z,self.p.orientation.w))
		yaw = euler[2]

		#Computing and updating the position on the pose object
		self.p.position.x += (self.r*(self.wr+self.wl)/2*self.dt*cos(yaw))
		self.p.position.y += (self.r*(self.wr+self.wl)/2*self.dt*sin(yaw))

		if(yaw>pi or yaw<-pi):
			yaw*=-1
		
		#Update yaw
		yaw+=self.r*(self.wr-self.wl)/self.l*self.dt
		#Send yaw value to ros log
		rospy.loginfo(yaw)
		#Compute the new quaternion with new yaw value
		quat = quaternion_from_euler(0, 0, yaw)
		#Update orientation 
		self.p.orientation.x = quat[0]
		self.p.orientation.y = quat[1]
		self.p.orientation.z = quat[2]
		self.p.orientation.w = quat[3]
		#self.posPub.publish(self.p)

	#Send transformation using broadcaster
	def broadcastTransform(self):
		#Initialize broadcaster
		br = TransformBroadcaster()
		t = TransformStamped()
		#Fill the transform with the position and orientations
		t.header.stamp = rospy.Time.now()
		#Frame names
		t.header.frame_id = "world"
		t.child_frame_id = "robot"
		t.transform.translation = Vector3(self.p.position.x, self.p.position.y,self.p.position.z)
		t.transform.rotation = self.p.orientation
		#Send transform
		br.sendTransform(t)
	
	#Restart pose
	def restart(self,_):
		self.p = Pose()

	#Get the pose directly
	def getPose(self):
		return self.p


def main():
	#Init the of odometry
	rospy.init_node('Odometry', anonymous=True)
	#Set rospy rate
	r = rospy.Rate(100)
	#Iniatilize class
	odometria = Odom()
	#Get speed from topic using the Odometry class methods
	rospy.Subscriber("/wl", Float32, odometria.lSpeed)
	rospy.Subscriber("/wr", Float32, odometria.rSpeed)
	rospy.Subscriber("position/restart",Empty, odometria.restart)

	#Keep calculating the position and sending the transform
	while not rospy.is_shutdown():
		odometria.calculate()
		odometria.broadcastTransform()
		r.sleep()


if __name__ == "__main__":
    main()
