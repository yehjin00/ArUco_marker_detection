#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('cvbridge_tutorials')  # 'cvbridge_tutorials' is package name
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2.aruco as aruco
import yaml
import os
from geometry_msgs.msg import Pose


class image_converter:

  def __init__(self):
    self.pose_pub = rospy.Publisher("pose",Pose,queue_size = 1)  # publisher's topic name can be anything
    self.image_pub = rospy.Publisher("image", Image, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback,queue_size = 1)  # '/usb_cam/image_raw' is usb_cam's topic name
 
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

      # OpenCV's CLAHE (Contrast Limited Adaptive Histogram Equalization)
      gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)  # Change grayscale
      aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)  # Use 5x5 dictionary to find markers (5x5: number of rectangles inside the marker 250: id range 0~249 )
      
      clahe = cv2.createCLAHE(clipLimit=2, tileGridSize=(8,8))
      img_HE = clahe.apply(gray)

      dst = np.hstack((gray, img_HE))

      parameters = aruco.DetectorParameters_create()  # Marker detection parameters
      corners, ids, rejected_img_points = aruco.detectMarkers(dst, aruco_dict, parameters=parameters)

      p = Pose()
      if np.all(ids is None):  # If there are markers not found by detector
        print('Marker undetected')
      if np.all(ids is not None):   # If there are markers found by detector
        print('Marker detected')
        print(ids)      # print marker ID
        for i in range(0, len(ids)): # Iterate in markers
          rvec, tvec,markerPoints= aruco.estimatePoseSingleMarkers(corners[i], 0.1, matrix_coefficients, distortion_coefficients)  # markerLength width 0.1m


          (rvec - tvec).any()  # get rid of that nasty numpy value array error
          # inversePerspective
          R, _ = cv2.Rodrigues(rvec) # converts rotation vector to rotation matrix using Rodrigues transformation 
          r1=R[0]
          r2=R[1]
          r3=R[2]
          R = np.matrix(R).T  # transposition
          tvec1=np.reshape(tvec,(3,1))
          t1=tvec1[0]
          t2=tvec1[1]
          t3=tvec1[2]
          a1=np.append(r1,t1)  # add list
          a2=np.append(r2,t2)
          a3=np.append(r3,t3)
          a4=np.array([0,0,0,1])
          t=np.r_[[a1],[a2],[a3],[a4]]  # make 4x4 Homogeneous transformation matrix (rot + transl)
          print('t : ',t)  
          tt=np.linalg.inv(t)  # inverse
          print('tt :',tt)  
          ct=np.array([tt[0][3],tt[1][3],tt[2][3]])  # camera pose
          print('ct : ',ct)
          invRvec, _ = cv2.Rodrigues(R) # rotation's transposition = rotation's inverse
          
          aruco.drawDetectedMarkers(dst.copy(), corners, ids)  # Draw a square around the markers


          aruco.drawAxis(dst, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.05)  # Draw Axis, axis length 0.05m
          cv2.imshow("Image window_HE", dst)

          
          p.position.x = t1
          p.position.y = t2
          p.position.z = t3
          # Make sure the quaternion is valid and normalized
          p.orientation.x = 0.0
          p.orientation.y = 0.0
          p.orientation.z = 0.0
          p.orientation.w = 1.0


    except CvBridgeError as e:
      print(e)
    
    cv2.imshow("Image window_HE", dst)
    cv2.waitKey(3)  # imshow & waitKey is essential. if waitKey doesn't exist, imshow turns off immediately

    try:
      self.pose_pub.publish(p) # After working with opencv image, convert to ros image again and publish.
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(dst,"bgr8")) # After working with opencv image, convert to ros image again and publish.
      
    except CvBridgeError as e:
      print(e)


def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
  # yaml file load(matrix_coefficients, distortion_coefficients)
  with open('/home/sparo/ros/catkin_ws_aruco/src/aruco/src/image_web/ost1.yaml') as f:
    data=f.read()  # read loaded file
    vegetables = yaml.load(data,Loader=yaml.FullLoader)
    k=vegetables['K']
    d=vegetables['D']
    kd=k['kdata'] # kd is 3x3 matrix in yaml file
    kd=np.reshape(kd,(3,3))  # so reshape
    dd=d['ddata'] # dd is 1x5 matrix in yaml file, so not reshape
    matrix_coefficients=np.array(kd)
    distortion_coefficients=np.array(dd)
  f.close()
  # main function
  main(sys.argv)
