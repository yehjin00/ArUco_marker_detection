#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('cvbridge_tutorials')  # 'cvbridge_tutorials' is package name
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import math

class cmd_pose:

    def __init__(self):
        self.vel_pub=rospy.Publisher("cmd_vel",Twist,queue_size=1)  # no function in publisher
        self.pose_sub=rospy.Subscriber("pose",Pose,self.callback,queue_size=1)
    
    def callback(self,pose):
        # pose
        angular = math.atan2(pose.position.x, pose.position.z)
        linear = 0.35*math.sqrt(pose.position.x ** 2 + pose.position.z ** 2)
        # angular = 0.05*math.atan2(pose.position.x, pose.position.y)
        # linear = 0.05*math.sqrt(pose.position.x ** 2 + pose.position.y ** 2)
        print('angular : ',angular)
        print('linear : ',linear)
        #print('x orientation : ',pose.orientation.x)
        #print('y orientation : ',pose.orientation.y)
        #print('z orientation : ',pose.orientation.z)

        # # speed control
        if linear<0.1:
            linear=0.0
        elif linear>0.18:
            linear=linear*1.5
        if angular<-0.0:
            angular=angular*1.5
        elif angular>0.0:
            angular=angular*1.5

        # velocity
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = -angular
        
        try:
            self.vel_pub.publish(cmd)
        except KeyboardInterrupt:
            print("Shutting down")

        # far, fast / close, slow speed
        
        
        
if __name__=='__main__':
    rospy.init_node('cmd_pose', anonymous=True)
    ic = cmd_pose()
    try:
        rospy.spin() 
    except KeyboardInterrupt:
        print("Shutting down")