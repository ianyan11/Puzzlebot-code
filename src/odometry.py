#!/usr/bin/env python

from math import cos, sin, pi

import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Float32, Empty
from geometry_msgs.msg import Pose, TransformStamped, Vector3
import numpy as np

class Odom:
	p = Pose()
	wl = 0
	wr = 0
	dt = 0.01
	#posPub = rospy.Publisher('Position', Pose, queue_size = 10)
	r = 0.05 # Wheel radius, 0.05m
	l = 0.19 # Distance between robot wheels 0.19m

	def lSpeed(self, data):
		self.wl = data.data
	
	def rSpeed(self, data):
		self.wr = data.data
	
	def calculate(self):
		euler = euler_from_quaternion((self.p.orientation.x,self.p.orientation.y,self.p.orientation.z,self.p.orientation.w))
		yaw = euler[2]

		self.p.position.x += (self.r*(self.wr+self.wl)/2*self.dt*cos(yaw))
		self.p.position.y += (self.r*(self.wr+self.wl)/2*self.dt*sin(yaw))

		if(yaw>pi or yaw<-pi):
			yaw*=-1
			
		yaw+=self.r*(self.wr-self.wl)/self.l*self.dt
		rospy.loginfo(yaw)
		quat = quaternion_from_euler(0, 0, yaw)
		self.p.orientation.x = quat[0]
		self.p.orientation.y = quat[1]
		self.p.orientation.z = quat[2]
		self.p.orientation.w = quat[3]
		#self.posPub.publish(self.p)

	def broadcastTransform(self):
		br = TransformBroadcaster()
		t = TransformStamped()
		t.header.stamp = rospy.Time.now()
		t.header.frame_id = "world"
		t.child_frame_id = "robot"
		t.transform.translation = Vector3(self.p.position.x, self.p.position.y,self.p.position.z)
		t.transform.rotation = self.p.orientation
		br.sendTransform(t)
		
	def restart(self,_):
		self.p = Pose()

	def getPose(self):
		return self.p


def main():
	rospy.init_node('Odometry', anonymous=True)
	r = rospy.Rate(100)
	odometria = Odom()
	rospy.Subscriber("/wl", Float32, odometria.lSpeed)
	rospy.Subscriber("/wr", Float32, odometria.rSpeed)
	rospy.Subscriber("position/restart",Empty, odometria.restart)

	while not rospy.is_shutdown():
		odometria.calculate()
		odometria.broadcastTransform()
		r.sleep()


if __name__ == "__main__":
    main()
