'''
To get camera intrinsic matrix and distortion coefficients
input: chessboard color frame
output:
'''

import cv2
import numpy as np
import glob
import scipy.io as scio
import argparse
from pathlib import Path

def calibrate(img_dir):
    # Define the dimensions of chessboard
    chessboard = (8,6)
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Create list to store vectors of 3D points in the world coordinate(X,Y,Z)
    objpoints = []
    # Create list to store vectors of 2D points in the image coordinate(u,v)
    imgpoints = []

    # 3D points in the world coordinate, like (0,0,0), (1,0,0), (2,0,0), ..., (0,1,0), (1,1,0),..., (7,5,0)
    objp = np.zeros((1, chessboard[0] * chessboard[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:chessboard[0], 0:chessboard[1]].T.reshape(-1, 2)

    # Find chessboard corner in each image in a given directory
    images = list(map(str, [p for p in Path(str(img_dir)).glob("*.png")]))
    # images = glob.glob('./position_calib/*.png')
    for fname in images:
        img = cv2.imread(fname)
        grayimg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners

        # If desired number of corners are found in the image then ret = true
        ret, corners = cv2.findChessboardCorners(grayimg, chessboard, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

        if ret == True:
            objpoints.append(objp)
            # Refine the corners which are found in each image
            corners2 = cv2.cornerSubPix(grayimg, corners, (11,11),(-1,-1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners on each chessboard
            img = cv2.drawChessboardCorners(img, chessboard, corners2, ret)
            cv2.imshow('img',img)
            cv2.waitKey(0)

    cv2.destroyAllWindows()

    # Calibrate camera
    ret, intrinsic, distcoeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, grayimg.shape[::-1], None, None)

    return intrinsic, distcoeffs

def save_camera_matrix(intrinsic, distcoeffs, depth_scale, output_name):
    camera_meta = {"intrinsic": intrinsic, "distcoeffs": distcoeffs, "depth_scale": float(depth_scale)}
    scio.savemat(f'{output_name}.mat', camera_meta)
    print("camera matrix written!")
    print("\n")


if __name__=='__main__':

    parser = argparse.ArgumentParser(prog='camcalib.py', description='Calibrate camera with opencv')
    parser.add_argument('--image_dir', '-img', help='Checkerboard image directory')
    parser.add_argument('--depth_scale', '-s', required=False, help='Depth scale')
    parser.add_argument('--output_name', '-o', required=False, help='Name the camera meta')

    args = parser.parse_args()
    # print(args)

    image_dir = args.image_dir
    depth_scale = args.depth_scale
    print(depth_scale)
    output_name = args.output_name

    Intrinsic, Distcoeffs = calibrate(image_dir)
    print("Intrinsic matrix:")
    print(Intrinsic)
    print("\n")
    print("Distortion coefficients:")
    print(Distcoeffs)
    print("\n")

    save_camera_matrix(Intrinsic, Distcoeffs, depth_scale, output_name)
