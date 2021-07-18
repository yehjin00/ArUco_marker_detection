# Aruco Marker Detection(with Webcam)

![](https://docs.opencv.org/3.4/markers.jpg)

## Prerequisites

- ROS
- Aruco Marker
 You can print marker from this site.
 https://chev.me/arucogen/

## Package Description
- aruco
 : for camera calibration and marker detection without ros
   test.py : code for marker detection without ros
 ```
 how to run test.py
 $ cd catkin_ws/src/aruco/src/
 $ python test.py
 ```

 * 'image_web' in aruco package : package for camera calibration
 ```
 how to run image_capture.py
 $ cd catkin_ws/src/aruco/src/image_web/
 $ python image_capture.py
 image capture using space key

 how to run camera_calibration.py
 $ cd catkin_ws/src/aruco/src/image_web/
 $ python cameracalibration.py --image_dir ~/catkin_ws/src/aruco/src/image_web --image_format png --prefix opencv_frame --square_size 25 --width 8 --height 6 --save_file ~/test/image_web/ost1.yaml
```

- cvbridge_tutorials
  : cvbridge example code and marker_detection code with ros
  ** caution! After doing camera calibration, do marker detection
  marker_detection.py : marker detection code with ros.

## How to build ArUco-marker-detection ROS package
```
Clone this project to your catkin's workspace src folder
 $ cd catkin_ws
 $ cd src
 $ git clone https://github.com/yehjin00/ArUco-marker-detection.git
 $ catkin_make
Move the packages in ArUco-marker-detection to $~/catkin_ws/src/

You only need to chmod the python file you want to use.
If you're going to calibrate, you need two files(image_capture.py, camera_calibration.py)
otherwise you only need one file(marker_detection.py).

Run marker Detection(in each different terminal)
 $ roscore
 $ roslaunch cvbridge_tutorials aruco.launch
```
