# The following code has been provided by openCV.
# The comments are made by us to help you with the calibration/undistort process.
# For the USB camera the effect was not very visible.
# For the CSI camera the effect was VERY visible.
import numpy as np
import cv2 as cv
import glob
import sys

import os
sys.path.append(os.path.relpath("../"))
from define.define import RobotCamera, CarCamera

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 

objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
 

objpoints = [] 
imgpoints = [] 

# Take around 10-20 photos with the camera (photos of a chess board at least 7x6)
# Move the photos in the csi/usb folder
# You will need to run this for each cameras with photos made by each camera.
images = glob.glob('photosCSI/*.jpg')
 
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    ret, corners = cv.findChessboardCorners(gray, (7,6), None)
    if ret == True:
        objpoints.append(objp)
 
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
 
        # Draw and display the corners
        cv.drawChessboardCorners(img, (7,6), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(10000)
 
cv.destroyAllWindows()


ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Choose one photo to undistort to see the difference (doesn't matter which one)
img = cv.imread('photosCSI/01.jpg')
h,  w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

np.savez('camera_matrix_csi.npz', arr=newcameramtx)
np.savez('dist_csi.npz', arr=dist)
np.savez('mtx_csi.npz', arr=mtx)
np.savez('tvecs_csi.npz', arr=tvecs)
np.savez('rvecs_csi.npz', arr=rvecs)

# Move the generated files to the matrix folder.
# Perhaps rename them or change the names in the savez functions.
# After this RobotCamera/CarCamera.load() can be called.
# Be sure the names in the load() are the same names from the matrix folder.



dst = cv.undistort(img, mtx, dist, None, newcameramtx)
 
# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv.imwrite('calibresult.png', dst)