import sys
try:
    import cv2
except:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
    import cv2

import numpy as np
import copy
import json

class HandEyeCalib:

    def load_CameraMatrix(self, use_sensor=False, filename="data/cameraMatrix.npy"):
        if use_sensor:
            print("use sensor")
        else:
            self.cameraMatrix=np.load(filename)

    def Aruco_to_3dpoint(self,corn, markersize, distCoeffs):
        cameraMatrix=self.cameraMatrix
        pose1 = cv2.aruco.estimatePoseSingleMarkers(corn, markersize, cameraMatrix, distCoeffs)
        rvec = pose1[0]
        tvec = pose1[1]
        mrv, jacobian = cv2.Rodrigues(rvec)
        markercorners = np.matmul(mrv, pose1[2].reshape(4, 3).T).T + np.tile(tvec.flatten(), [4, 1])
        markercorners = markercorners.reshape([4, 3])
        return markercorners

    def Calibration_EyeinHand_SingleImg(self,rgb,flag_show=False):
        display_img = copy.deepcopy(rgb)
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(display_img, arucoDict,
                                                           parameters=arucoParams)
        sortcorners=np.array(corners)[np.argsort(ids.flatten())]
        for index in range(0,ids.__len__()):
            point1=corners[index][0][0]
            cv2.putText(display_img, "[ "+str(ids[index])+" ]", point1.astype(np.int).tolist(), 1, 1, (0, 0,255))

        for cpoint_pixel in  sortcorners.reshape(ids.__len__() * 4, 2):
            cv2.circle(display_img, cpoint_pixel.astype(np.int), 5, (255,0,0 ),-1)

        if flag_show:
            cv2.imshow("detected points", display_img)
            cv2.waitKey(0)

        src=[[[-11,-12,0],[-7,-12,0],[-7,-8,0],[-11,-8,0]],#0
             [[-5,-12,0],[-1,-12,0],[-1,-8,0],[-5,-8,0]],#1
             [[1,-12,0],[5,-12,0],[5,-8,0],[1,-8,0]],#2
             [[7,-12,0],[11,-12,0],[11,-8,0],[7,-8,0]],#3
             [[-11,-6,0],[-7,-6,0],[-7,-2,0],[-11,-2,0]],#4
             [[-5,-6,0],[-1,-6,0],[-1,-2,0],[-5,-2,0]],#5
             [[1,-6,0],[5,-6,0],[5,-2,0],[1,-2,0]],#6
             [[7,-6,0],[11,-6,0],[11,-2,0],[7,-2,0]]#7
             ]

        src=np.array(src)
        src=np.array(src)[np.sort(ids.flatten())]
        distCoeffs = np.array([0, 0, 0, 0, 0])
        dst=[]
        for cor in sortcorners:
            cor3d=self.Aruco_to_3dpoint(cor,40,distCoeffs)
            dst.append(cor3d)
        dst = np.array(dst).reshape(4 * corners.__len__(), 3)
        src = np.array(src).reshape(4 * corners.__len__(), 3)*10
        RT=cv2.estimateAffine3D(src,dst)
        R=RT[1][::,0:3]
        T=RT[1][::,3]
        return R,T.tolist(),display_img

    def write_calib(self,calibration,filename="calibration.json"):
        calibration_data = {"calibration_data": calibration}
        with open(filename, "w") as json_file:
            json.dump(calibration_data, json_file)

    def load_calibration(self,filename='calibration.json'):
        with open(filename) as json_file:
            json_data = json.load(json_file)
            calibration_data=json_data["calibration_data"]
        return calibration_data
