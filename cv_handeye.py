import cv2
import numpy as np
import glob
import math
import scipy.io as scio
import argparse
from pathlib import Path

def find_extrinsic(cam_dir, img_dir):
    # Define the dimensions of chessboard
    chessboard = (8,6)
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    rvec_list = []
    tvec_list = []

    camera_meta = scio.loadmat(cam_dir)
    # print(camera_meta)
    rgb_intrinsic = camera_meta['intrinsic']
    rgb_distcoeffs = camera_meta['distcoeffs']

    # 3D points in the world coordinate, like (0,0,0), (1,0,0), (2,0,0), ..., (0,1,0), (1,1,0),..., (7,5,0)
    objp = np.zeros((1, chessboard[0] * chessboard[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:chessboard[0], 0:chessboard[1]].T.reshape(-1, 2)

    # Find chessboard corner in each image in a given directory
    images = list(map(str, [p for p in Path(str(img_dir)).glob("*.png")]))
    images = sorted(images, key=lambda i: int(i.split("_")[-1].replace(".png", "")))
    print ("Image order:", images)
    print("\n")

    # images = glob.glob('./position_calib/*.png')
    for fname in images:
        img = cv2.imread(fname)
        grayimg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        # If desired number of corners are found in the image then ret = true
        ret, corners = cv2.findChessboardCorners(grayimg, chessboard, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

        if ret == True:
            # Refine the corners which are found in each image
            corners2 = cv2.cornerSubPix(grayimg, corners, (11,11),(-1,-1), criteria)
            imgp = corners2.reshape(-1, 2)

            # Draw and display the corners on each chessboard
            img = cv2.drawChessboardCorners(img, chessboard, corners2, ret)
            cv2.imshow('checkerboard_calibrate',img)
            cv2.waitKey(0)

        ret, rvecs, tvecs = cv2.solvePnP(objp, corners2, rgb_intrinsic, rgb_distcoeffs)
        rvec_list.append(rvecs)
        tvec_list.append(tvecs)

    cv2.destroyAllWindows()

    return rvec_list, tvec_list

# Convert euler angle to rotation matrix
def euler_to_rotmx(theta) :
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])

    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])

    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])

    R = np.dot(R_z, np.dot( R_y, R_x ))
    return R

def get_RT_chess_to_cam(rvecs, tvecs):
    '''
        From chessboard coordinate convert to camera coordinate,
        brings points from the camera coordinate to the chessboard coordinate.
        Input：
            rvecs:  SolvePNP 得到的 rotation vectors (需要透過 Rodrigues 轉換)。
            tvecs: SolvePNP 得到的 tramslation vectors。
        Output:
            R_chess2cam_kinnect:
            T_chess2cam_kinnect
    '''
    # convert opencv camera to kinect V1 camera
    rotZ_to_kinnect = np.eye(3)
    rotZ_to_kinnect[0,0] = -1
    rotZ_to_kinnect[1,1] = -1

    # uss Rodrigues convert rotation vectors to rotation matrix
    R_cam_to_chess = cv2.Rodrigues(rvecs)
    R_cam_to_chess = R_cam_to_chess[0]

    # convert rotation matrix from chessboard to kinnect V1 camera
    R_chess_to_cam = np.linalg.inv(R_cam_to_chess)
    R_chess2cam_kinnect = np.dot(rotZ_to_kinnect, R_chess_to_cam)

    # convert translation vectors from chessboard to kinecvt V1 camera
    T_cam2chess_in_mm = 40*tvecs # chessboard length 40mm
    T_chess2cam = -1 *( np.dot(R_chess_to_cam, T_cam2chess_in_mm))
    T_chess2cam_kinnect = np.dot(rotZ_to_kinnect, T_chess2cam)

    return R_chess2cam_kinnect, T_chess2cam_kinnect

def get_R_robot_to_tool(degree):
    '''
        Input:
            theta_in_degree: in degree
            tvecs_im_mm:
        Output:

    '''
    # assert(lens(theta_in_degree)%3 == 0)

    theta_in_rad = degree * (math.pi / 180)
    R_robot2tool = euler_to_rotmx(theta_in_rad)
    print("rrrrrrrr")
    print(R_robot2tool)

    return R_robot2tool

def handeyecalib(R_all_chess_to_cam, T_all_chess_to_cam, R_all_base_to_tool, T_all_base_to_tool):
    R_cam_to_base, T_cam_to_base = cv2.calibrateHandEye(R_all_chess_to_cam, T_all_chess_to_cam, R_all_base_to_tool, T_all_base_to_tool)
    H_cam_to_base = np.column_stack((R_cam_to_base, T_cam_to_base))
    H_cam_to_base = np.row_stack((H_cam_to_base, np.array([0, 0, 0, 1])))
    return H_cam_to_base, R_cam_to_base, T_cam_to_base

def save_transform_matrix(H, R, T, output_name):
    transform_meta = {"Rotation matrix": R, "Translation matrix": T, "Transform matrix": H}
    scio.savemat(f'{output_name}.mat', transform_meta)
    print("transform matrix written!")
    print("\n")

def main(cam_mtx_dir, image_dir):
    #計算end to base變換矩陣
    theta_in_degree=[]
    R_all_robot_to_tool=[]
    T_all_robot_to_tool=[]

    #計算board to cam 變換矩陣
    R_all_chess_to_cam=[]
    T_all_chess_to_cam=[]

    rot_vecs, trans_vecs = find_extrinsic(cam_mtx_dir, image_dir)

    print("Rotation vectors:")
    print(rot_vecs)
    print("\n")
    print("Translation vectors:")
    print(trans_vecs)
    print("\n")
    # print(trans_vecs[0])
    # print(trans_vecs[1])

    for n in range(len(tool_pose)):
        tp_arr = np.array(tool_pose[n][1][0:3])
        print(tp_arr)
        T_all_robot_to_tool.append(tp_arr)

        theta_arr = np.array(tool_pose[n][1][3:6])
        print(theta_arr)
        theta_in_degree.append(theta_arr)
        R_robot2tool = get_R_robot_to_tool(theta_in_degree[n])
        R_all_robot_to_tool.append(R_robot2tool)

        R_chess2cam, T_chess2cam = get_RT_chess_to_cam(rot_vecs[n], trans_vecs[n])
        R_all_chess_to_cam.append(R_chess2cam)
        T_all_chess_to_cam.append(T_chess2cam)

    # print(T_all_robot_to_tool)
    # print(theta_in_degree)
    print("--------------")
    print(R_all_chess_to_cam)
    print(T_all_chess_to_cam)
    print(R_all_robot_to_tool)
    print(T_all_robot_to_tool)
    H_cam_to_base, R_cam_to_base, T_cam_to_base = handeyecalib(R_all_chess_to_cam, T_all_chess_to_cam, R_all_robot_to_tool, T_all_robot_to_tool)

    print("R_cam_to_base:")
    print(R_cam_to_base)
    print("\n")
    print("T_cam_to_base:")
    print(T_cam_to_base)
    print("\n")
    print('Hcg_cam_to_base')
    print(H_cam_to_base)
    print("\n")

    save_transform_matrix(H_cam_to_base, R_cam_to_base, T_cam_to_base, output_name)

if __name__=='__main__':

    parser = argparse.ArgumentParser(prog='cv_handeye.py', description='Calibrate Hand-Eye Coordinate system with opencv')
    parser.add_argument('--image_dir', '-img', help='Checkerboard image directory')
    parser.add_argument('--cam_mtx_path', '-cam', required=False, help='Camera matrix path')
    parser.add_argument('--tool_pose_path', '-tp', required=False, help='Tool pose path')
    for i in range(10):
        parser.add_argument(f'--tool{i}', f'-t{i}', nargs=6, help='value:x y z w p r')
    parser.add_argument('--output_name', '-o', required=False, help='Name the transform matrix meta')

    args = parser.parse_args()
    # print(args)

    image_dir = args.image_dir
    cam_mtx_dir = args.cam_mtx_path
    tool_pose_dir = args.tool_pose_path
    output_name = args.output_name

    tool_pose = [(num, list(map(lambda val: float(val), val))) for num, val in vars(args).items() if val != None and "tool" in num]
    # print(tool_pose)
    print(f'get {len(tool_pose)} tool poses')
    print("\n")

    main(cam_mtx_dir, image_dir)
