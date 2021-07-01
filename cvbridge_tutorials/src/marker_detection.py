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
      gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)  # Change grayscale
      aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)  # Use 5x5 dictionary to find markers
      parameters = aruco.DetectorParameters_create()  # Marker detection parameters
      corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
      if np.all(ids is None):  # If there are markers found by detector
            print('Marker undetected')
      if np.all(ids is not None):   # If there are markers found by detector
          print('Marker detected')
          print(ids)
          for i in range(0, len(ids)): # Iterate in markers
              rvec, tvec= aruco.estimatePoseSingleMarkers(corners[i], 0.1, matrix_coefficients, distortion_coefficients)  # markerLength width 0.1m
              (rvec - tvec).any()  # get rid of that nasty numpy value array error
              crvec,_=cv2.Rodrigues(rvec)
              crvect=np.matrix(crvec).T
              print('rvec : ',rvec,rvec.shape)
              print('crvec : ',crvec,crvec.shape)
              print('crvect : ',crvect,crvect.shape)
              print('marker tvec : ',tvec)
              print('camera tvec : ',-tvec)
              aruco.drawDetectedMarkers(cv_image.copy(), corners, ids)  # Draw A square around the markers
              aruco.drawAxis(cv_image, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.05)  # Draw Axis, axis length 0.05m
              if (tvec[0][0][0]>0.01):
                  cv2.arrowedLine(cv_image, (490, 240), (590, 240), (138,43,226), 3)
              elif (tvec[0][0][0]<-0.01):
                  cv2.arrowedLine(cv_image, (150, 240), (50, 240), (138,43,226), 3)
              if (tvec[0][0][0]>0 and tvec[0][0][0]<0.02):
                  print('-------------------------------------------------------')
                  break
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
  matrix_coefficients = np.array([[ 4.9350028035371434e+02, 0., 3.1889472537064762e+02], [0.,
       4.9752379240241839e+02, 2.3323580951832940e+02], [0., 0., 1.] ])
  distortion_coefficients = np.array([ 1.3514513045692980e-01, -4.8663060594638929e-01,
       6.3572242938879548e-04, 5.6972282484044220e-04, 5.4433200932025450e-01 ])
  main(sys.argv)
