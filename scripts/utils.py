import sys
sys.path.append("../ketirobotsdk")
from ketirobotsdk.sdk import *
from time import *
import threading
import math
sys.path.append("../include/gripper")
from keti_zimmer_gripper import KetiZimmer
import numpy as np


def moveL_command(R, T): # R : [0 0 0, 0 0 0, 0 0 0], T : [x, y, z]
    cmd_mat = [0]*16
    cmd_mat[0:3] = R[0]
    cmd_mat[4:7] = R[1]
    cmd_mat[8:11] = R[2]
    cmd_mat[3] = T[0]
    cmd_mat[7] = T[1]
    cmd_mat[11] = T[2]
    cmd_mat[15] = 1
    
    movel(Base, cmd_mat)
    
    
def moveB_command(n, Rs,Ts):
    r = 0.02
    waypoints = []
    print(Ts)
    for i in range(n):
        cmd_mat = [0]*16
        cmd_mat[0:3] = Rs[i][0]
        cmd_mat[4:7] = Rs[i][1]
        cmd_mat[8:11] = Rs[i][2]
        cmd_mat[3] = Ts[i][0]
        cmd_mat[7] = Ts[i][1]
        cmd_mat[11] = Ts[i][2]
        cmd_mat[15] = 1
        
        waypoints.append(cmd_mat)

        print(waypoints)
    if n == 1:
        moveb(Base, r, 1, waypoints[0])
    elif n == 2:
        moveb(Base, r, 2, waypoints[0], waypoints[1])
    elif n == 3:
        moveb(Base, r, 3, waypoints[0], waypoints[1], waypoints[2])
    elif n == 4:
        moveb(Base, r, 4, waypoints[0], waypoints[1], waypoints[2], waypoints[3])
    elif n == 5:
        moveb(Base, r, 5, waypoints[0], waypoints[1], waypoints[2], waypoints[3], waypoints[4])
    else :
        print("error n size!!!")

        

def transform_point1(transformation_matrix, point):
    '''
    transformation_matrix = np.array([[1, 0, 0, 2],
                                    [0, 1, 0, 3],
                                    [0, 0, 1, 4],
                                    [0, 0, 0, 1]])
    point = np.array([1, 2, 3])
    '''
    homogeneous_point = np.append(point, 1)  # 좌표를 확장하여 동차 좌표로 변환
    transformed_point = np.dot(transformation_matrix, homogeneous_point)  # 변환 행렬과 좌표를 행렬 곱셈
    transformed_point = transformed_point[:3] / transformed_point[3]  # 동차 좌표를 일반 좌표로 변환
    
    return transformed_point


def transform_point2(transformation_matrix1, transformation_matrix2, point):
    homogeneous_point = np.append(point, 1)  # 좌표를 확장하여 동차 좌표로 변환
    transformed_point = np.dot(transformation_matrix2, np.dot(transformation_matrix1, homogeneous_point))  # 두 개의 변환 행렬과 좌표를 행렬 곱셈
    transformed_point = transformed_point[:3] / transformed_point[3]  # 동차 좌표를 일반 좌표로 변환
    
    return transformed_point




