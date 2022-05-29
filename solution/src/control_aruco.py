#!/usr/bin/env python3
from grpc import RpcContext
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist, Vector3


class Control_vis:
    #Publisher of velocities
    posPub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    msg = Twist()
    #PID constants
    kp_l = 0.000005
    kp_a = -0.0003    

    light_state = String()

    def ligthState(self, data):
        """Method to asing the ligth state to class variable"""
        self.light_state = data

    def moveToObjective (self, data: Vector3):
        
        if (self.light_state.data == "red_light"):
            print("Semaforo rojo")
            self.posPub.publish(Twist())
            return

        elif(data != Vector3()):
            self.msg.linear.x = self.kp_l*(50000-data.z)
            self.msg.angular.z = self.kp_a*(data.x)
            self.posPub.publish(self.msg)
        else:
            self.posPub.publish(Twist())


def main():
    rospy.init_node('ArucoControl', anonymous=True)
    
    control = Control_vis()
    #Subscribers
    rospy.Subscriber('/light_state', String, control.ligthState)
    rospy.Subscriber('/marker_properties',Vector3, control.moveToObjective)
    rospy.Subscriber('/aruco/pose', Vector3, control.moveToObjective)
    rospy.spin()
        
if __name__ == "__main__":
    main()
