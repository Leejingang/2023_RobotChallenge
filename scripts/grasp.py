import numpy as np
import math as m
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class selecting():
    
    def __init__(self, width, hight, depth_data, intr_params):
        self.width_param = width
        self.hight_param = hight
        self.depth_data = depth_data
        self.intr_params = intr_params
        
    def distance_fiter(self, grips):
        #depth_data : rgb_data, depth_data = sensor.get_data()
        # sensor = RSSensor()
        # sensor.start()
        # ppx, ppy, fx, fy = sensor.intr_params.ppx, sensor.intr_params.ppy, sensor.intr_params.fx, sensor.intr_params.fy
        
        spatial_grips = []
        
        for i in grips:
            #center
            center_Xc = i[0]
            center_Yc = i[1]
            center_Zc = self.depth_data[center_Yc,center_Xc]
            center_point_x = (center_Xc-self.intr_params[0])/self.intr_params[2]*center_Zc
            center_point_y = (center_Yc-self.intr_params[1])/self.intr_params[3]*center_Zc
            center_point_z = float(center_Zc)
            
            #left
            left_Xc = i[6]
            left_Yc = i[7]
            left_Zc = self.depth_data[left_Yc,left_Xc]
            left_point_x = (left_Xc-self.intr_params[0])/self.intr_params[2]*left_Zc
            left_point_y = (left_Yc-self.intr_params[1])/self.intr_params[3]*left_Zc
            left_point_z = float(left_Zc)
            
            #right
            right_Xc = i[8]
            right_Yc = i[9]
            right_Zc = self.depth_data[right_Yc,right_Xc]
            right_point_x = (right_Xc-self.intr_params[0])/self.intr_params[2]*right_Zc
            right_point_y = (right_Yc-self.intr_params[1])/self.intr_params[3]*right_Zc
            right_point_z = float(right_Zc)
            
            #girp width
            width = m.sqrt((left_point_x - right_point_x)**2 + (left_point_y - right_point_y) ** 2 + (left_point_z - right_point_z) ** 2)
            hight = abs(left_point_z = right_point_x)
            
            
            #filtering
            if self.width_param < width and self.hight_param < hight:
                
            # [[center], [left], [right], [width, hight], score]
                spatial_grips.append([[center_point_x, center_point_y, center_point_z],
                                      [left_point_x, left_point_y, left_point_z],
                                      [right_point_x, right_point_y, right_point_z],
                                      [width, hight],
                                      i[10]])
            
        if len(spatial_grips) > 0:
            spatial_grips = sorted(spatial_grips, key=lambda x: x[10])
            return spatial_grips
        
        else :
            return 0
        
    def optimal_select(self, grips):
        
        grips = self.distance_fiter(grips)
        
        if grips == 0:
            return -1
        
        else :
            grips = sorted(grips, key=lambda x: x[4])

            return grips[-1]
        

class normal_vector():

    def fit_plane_to_points_ransac(self, points, threshold=0.1, max_iterations=1000):
        # 데이터를 Nx3 크기의 배열로 변환
        data = np.array(points)

        # x, y, z 좌표를 따로 분리
        x, y, z = data[:, 0], data[:, 1], data[:, 2]

        # A 행렬을 구성합니다.
        A = np.column_stack((x, y, np.ones_like(x)))

        best_plane = (0, 0, 0, 0)  # 최적의 평면 방정식을 저장할 변수 초기화
        best_inliers = np.array([])  # 인라이어 포인트들을 저장할 변수 초기화
        best_num_inliers = 0  # 최적의 인라이어 개수 초기화

        for _ in range(max_iterations):
            # 무작위로 3개의 점을 선택하여 평면 방정식을 추정
            random_indices = np.random.choice(len(data), 3, replace=False)
            sample_points = data[random_indices]
            A_sample = np.column_stack((sample_points[:, 0], sample_points[:, 1], np.ones(3)))
            _, _, D = np.linalg.lstsq(A_sample, sample_points[:, 2], rcond=None)[0]
            normal_vector = np.cross(sample_points[1] - sample_points[0], sample_points[2] - sample_points[0])
            A, B, C = normal_vector

            # 추정한 평면 방정식을 이용하여 모든 점들과의 거리를 계산
            distances = np.abs(A * x + B * y + C * z - D) / np.sqrt(A**2 + B**2 + C**2)

            # 임계값(threshold) 이내의 점들을 인라이어로 간주
            inliers = data[distances < threshold]
            num_inliers = len(inliers)

            # 현재 추정한 평면이 최적인 경우, 변수들을 업데이트
            if num_inliers > best_num_inliers:
                best_plane = (A, B, C, D)
                best_inliers = inliers
                best_num_inliers = num_inliers

        return best_plane
    
    def plane_visualization(self, points):

        # 최적의 평면을 RANSAC 알고리즘으로 구함.
        A, B, C, D = self.fit_plane_to_points_ransac(points)

        print(f"평면 방정식: {A:.2f}x + {B:.2f}y + {C:.2f}z + {D:.2f} = 0")

        # 시각화를 위해 평면 방정식의 x, y, z에 해당하는 좌표들을 생성
        x_plane, y_plane = np.meshgrid(np.arange(-10, 30, 1), np.arange(-10, 30, 1))
        z_plane = (-A * x_plane - B * y_plane - D) / C

        # 3D 그래프를 생성
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # 3D 점 추가
        x, y, z = np.array(points).T
        ax.scatter(x, y, z, c='b', marker='o', label='데이터 포인트')

        # 평면 추가
        ax.plot_surface(x_plane, y_plane, z_plane, alpha=0.5)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('RANSAC plane')

        plt.show()