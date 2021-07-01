#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('cvbridge_tutorials')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2.aruco as aruco
import yaml


class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("cvbridge_image",Image,queue_size = 1)
    print("published to /camera/image")

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback,queue_size = 1)
    print("subscribed to /camera/image")
 
  def callback(self,data):
    print('callback')
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    except CvBridgeError as e:
      print(e)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)