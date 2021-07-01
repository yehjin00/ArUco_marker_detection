#!/usr/bin/env python
import numpy as np
import cv2
import cv2.aruco as aruco  # solved Error: $ pip install opencv-contrib-python
import yaml


cap = cv2.VideoCapture(0)  # Get the camera source

count = 1  # Count images



def track(matrix_coefficients, distortion_coefficients):
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH )
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT )
    
    print('mm',matrix_coefficients)
    global count
    while True:
        ret, frame = cap.read()
        count = count + 1
        print('{}th image'.format(count))

        # operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)  # Use 5x5 dictionary to find markers
        parameters = aruco.DetectorParameters_create()  # Marker detection parameters
# lists of ids and the corners beloning to each id
        corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        print(ids)
        if np.all(ids is None):  # If there are markers found by detector
            print('Marker undetected')
        if np.all(ids is not None):  # If there are markers found by detector
            print('Marker detected')
            print(len(ids))
            for i in range(0, len(ids)):  # Iterate in markers
             # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
                rvec, tvec= aruco.estimatePoseSingleMarkers(corners[i], 0.1, matrix_coefficients, distortion_coefficients)  # markerLength width 0.1m
                print('rvec', rvec)
                print('tvec', tvec)
                (rvec - tvec).any()  # get rid of that nasty numpy value array error
                
                aruco.drawDetectedMarkers(frame.copy(), corners, ids)  # Draw A square around the markers
                aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.05)  # Draw Axis, axis length 0.05m
                print('tvec[0]', tvec[0][0][0])
                print('width : ',width)
                print('height : ',height)
                if (tvec[0][0][0]>0.01):
                    cv2.arrowedLine(frame, (490, 240), (590, 240), (138,43,226), 3)
                elif (tvec[0][0][0]<-0.01):
                    cv2.arrowedLine(frame, (150, 240), (50, 240), (138,43,226), 3)
                if (tvec[0][0][0]>0 and tvec[0][0][0]<0.02):
                    print('-------------------------------------------------------')
                    break
        # Display the resulting frame
        print('show image')
        
        cv2.imshow('frame', frame)
        # Wait 3 milisecoonds for an interaction. Check the key and do the corresponding job.
        key = cv2.waitKey(3) & 0xFF
        if key == ord('q'):  # Quit
            break
	        
        
		 
        # When everything done, release the capture


if __name__ == '__main__':
    matrix_coefficients = np.array([[ 4.9350028035371434e+02, 0., 3.1889472537064762e+02], [0.,
       4.9752379240241839e+02, 2.3323580951832940e+02], [0., 0., 1.] ])
    distortion_coefficients = np.array([ 1.3514513045692980e-01, -4.8663060594638929e-01,
       6.3572242938879548e-04, 5.6972282484044220e-04, 5.4433200932025450e-01 ])
    print(matrix_coefficients)
    track(matrix_coefficients, distortion_coefficients)
    print(matrix_coefficients)
    cap.release()
    cv2.destroyAllWindows()
 

