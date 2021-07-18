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


class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("cvbridge_image",Image,queue_size = 1)  # publisher's topic name can be anything
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback,queue_size = 1)  # '/usb_cam/image_raw' is usb_cam's topic name
 
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
      gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)  # Change grayscale
      aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)  # Use 5x5 dictionary to find markers  (5x5: number of rectangles inside the marker  250: id range 0~249 )
      parameters = aruco.DetectorParameters_create()  # Marker detection parameters
      corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
      if np.all(ids is None):  # If there are markers not found by detector
        print('Marker undetected')
      if np.all(ids is not None):   # If there are markers found by detector
        print('Marker detected')
        print(ids)      # print marker ID
        for i in range(0, len(ids)): # Iterate in markers
          rvec, tvec= aruco.estimatePoseSingleMarkers(corners[i], 0.1, matrix_coefficients, distortion_coefficients)  # markerLength width 0.1m
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
          invRvec, _ = cv2.Rodrigues(R)
          aruco.drawDetectedMarkers(cv_image.copy(), corners, ids)  # Draw A square around the markers
          aruco.drawAxis(cv_image, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.05)  # Draw Axis, axis length 0.05m
          if (ct[0]<-0.02):  # Let know direction
            cv2.arrowedLine(cv_image, (490, 240), (590, 240), (138,43,226), 3)  # image, start point, final point, color, size
          elif (ct[0]>0.02):
            cv2.arrowedLine(cv_image, (150, 240), (50, 240), (138,43,226), 3)
          if (ct[0]>0 and ct[0]<0.02):
            print('-------------------------------------------------------')
          if (ct[0]<-0.02 and ct[0]>0.02 and invRvec[0][0]<3.2 and invRvec[0][0]>3 and invRvec[1][0]<0.01 and invRvec[1][0]>-0.01):
            str='Front'
            cv2.putText(cv_image,str,(280,100),cv2.FONT_HERSHEY_PLAIN,3,(138,43,226),3)  # image, text, position, font, size, color, thickness
          str='o'  # display camera's center
          cv2.putText(cv_image,str,(320,240),cv2.FONT_HERSHEY_PLAIN,1,(138,43,226),3)  # image, text, position, font, size, color, thickness
          cv_image1 = draw_data(cv_image, invRvec[1], ct[0],ct[1])
          cv2.imshow("Image window", cv_image1)
          
    except CvBridgeError as e:
      print(e)
    
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
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

def draw_data(original_img,curv,center_dist1,center_dist2):  # draw real-time pose and rotation
  new_img=original_img
  h=new_img.shape[0]
  font=cv2.FONT_HERSHEY_DUPLEX
  text='Curve'+'{:04.2f}'.format(float(curv))+'degree'  # curve : how much is the y-axis rotated
  cv2.putText(new_img,text,(40,70),font,0.8,(255,255,255),1,cv2.LINE_AA)
  direction1=''
  direction2=''
  abs_center_dist1=abs(center_dist1)
  abs_center_dist2=abs(center_dist2)
  if center_dist1>0.02:
    direction1='left'
    text = '{:.2f}'.format(float(abs_center_dist1))+ 'm ' + direction1 + ' of center'
    cv2.putText(new_img,text,(40,120),font,0.8,(255,255,255),1,cv2.LINE_AA)
  elif center_dist1<-0.02:
    direction1='right'
    text = '{:.2f}'.format(float(abs_center_dist1))+ 'm ' + direction1 + ' of center'
    cv2.putText(new_img,text,(40,120),font,0.8,(255,255,255),1,cv2.LINE_AA)
  if center_dist2>0.02:
    direction2='down'
    text='{:.2f}'.format(float(abs_center_dist2))+ 'm ' + direction2 + ' of center'
    cv2.putText(new_img,text,(40,170),font,0.8,(255,255,255),1,cv2.LINE_AA)
  elif center_dist2<-0.02:
    direction2='up'
    text='{:.2f}'.format(float(abs_center_dist2))+ 'm ' + direction2 + ' of center'
    cv2.putText(new_img,text,(40,170),font,0.8,(255,255,255),1,cv2.LINE_AA)

  return new_img

if __name__ == '__main__':
  # yaml file load(matrix_coefficients, distortion_coefficients)
  with open('/home/sparo1/catkin_ws/src/aruco/src/image_web/ost1.yaml') as f:
    data=f.read()  # read loaded file
    vegetables = yaml.load(data,Loader=yaml.FullLoader)
    k=vegetables['K']
    d=vegetables['D']
    kd=k['kdata']
    kd=np.reshape(kd,(3,3))
    dd=d['ddata']
    matrix_coefficients=np.array(kd)
    distortion_coefficients=np.array(dd)
  f.close()
  # main function
  main(sys.argv)