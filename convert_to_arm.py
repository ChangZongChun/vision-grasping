import cv2
import numpy as np
import scipy.io as scio
import math
import argparse

# Checks if a matrix is a valid rotation matrix.
# def isRotationMatrix(R) :
#     Rt = np.transpose(R)
#     shouldBeIdentity = np.dot(Rt, R)
#     I = np.identity(3, dtype = R.dtype)
#     n = np.linalg.norm(I - shouldBeIdentity)
#     return n < 1e-6

def rotmx_to_euler_radians(R) :
    # assert(isRotationMatrix(R))
    sy = math.sqrt(R[1,2] * R[1,2] +  R[2,2] * R[2,2])
    x = math.atan2(-R[1,2] , R[2,2])
    y = math.atan2(R[2,0], sy)
    z = math.atan2(-R[0,1], R[0,0])

    return np.array([x, y, z])

def align_translation(H_cam_to_robot, traget_in_kinect):
    # bring Grasp Pose target point in camera frame to robot world frame
    H_robot_to_cam = np.linalg.inv(H_cam_to_robot)
    traget_in_robot = np.dot(H_robot_to_cam, traget_in_kinect)
    traget_in_robot = traget_in_robot.reshape(4, ).T

    return traget_in_robot

def align_rotation(R_cam_to_robot, R_kinect_to_object):
    # ------ Robot World frame to Grasp Pose Predict ---------
    R_object_to_gripper = np.array(
                            [[0, 0, 1],
                            [ 0, -1, 0],
                            [1, 0, 0]])

    R_robot_to_cam = np.linalg.inv(R_cam_to_robot)

    R_cam_to_gripper = np.dot(R_kinect_to_object, R_object_to_gripper)
    R_robot_to_gripper = np.dot(R_robot_to_cam, R_cam_to_gripper)
    prd_theta_in_rad = rotmx_to_euler_radians(R_robot_to_gripper)
    prd_theta_in_deg = prd_theta_in_rad * (180 / math.pi)

    return prd_theta_in_deg


def main(transform_mtx):

    # rotZ_to_kinnect = np.eye(3)
    # rotZ_to_kinnect[0,0] = -1
    # rotZ_to_kinnect[1,1] = -1

    # bring Grasp Pose target point in camera frame to robot world frame
    transform_meta = scio.loadmat(transform_mtx)
    # print(transform_meta)
    R_cam_to_robot = transform_meta['Rotation matrix']
    H_cam_to_robot = transform_meta['Transform matrix']

    R_test = np.array(
    [[-0.72762022,  0.68430807, -0.04786739],
    [ 0.6183957,   0.62413197, -0.47754167],
    [-0.29691004, -0.37706996, -0.87730421]])

    gg = np.load('gg.npy')
    # print(gg)

    R_cam_to_object = gg[1][4:13].reshape(3, 3)
    traget_in_cam = gg[1][13:16].reshape(-1, 3)

    # print(R_cam_to_object)
    # print(traget_in_cam)
    # print(H_cam_to_robot)

    # R_kinect_to_object = np.dot(rotZ_to_kinnect, R_cam_to_object)
    # traget_in_kinect = np.dot(rotZ_to_kinnect, traget_in_cam.T)
    # traget_in_kinect = np.concatenate((traget_in_kinect,[[1]]),0)

    traget_in_cam = np.concatenate((traget_in_cam,[1]), axis=None)

    traget_in_robot = align_translation(H_cam_to_robot, traget_in_cam.T)
    print("Grasp Pose in Robot World Frame:")
    print("x  y  z:")
    print(traget_in_robot[:3])
    print("\n")

    prd_theta_in_deg = align_rotation(R_cam_to_robot, R_cam_to_object)
    print("Grasp Pose in Robot World Frame:")
    print("w  p  r:")
    print(prd_theta_in_deg)
    print("\n")


if __name__=='__main__':
    parser = argparse.ArgumentParser(prog='get_camera_matrix.py', description='Get intrinsic parameters and build ccamera matrix')
    parser.add_argument('--transform_mtx', '-tf', help='Transform matrix path')
    args = parser.parse_args()

    transform_mtx = args.transform_mtx

    main(transform_mtx)
