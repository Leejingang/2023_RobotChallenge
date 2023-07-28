import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import rospy
from geometry_msgs.msg import PoseArray




class PointsPose(object):
	def __init__(self, topic="/mouse_click_points"):
		self.sub = rospy.Subscriber(topic, PoseArray, callback=self.callback)

		self.point = None

	def callback(self, msg):
		self.point = msg
  
  
  

def fit_plane_to_points(points):
    # 입력 데이터의 평균을 구합니다.
    centroid = np.mean(points, axis=0)
    
    # 입력 데이터에서 평균을 뺀 중심화된 데이터를 계산합니다.
    centered_points = points - centroid
    
    # 중심화된 데이터의 공분산 행렬을 계산합니다.
    covariance_matrix = np.dot(centered_points.T, centered_points)
    
    # 공분산 행렬의 고유값과 고유벡터를 계산합니다.
    eigenvalues, eigenvectors = np.linalg.eigh(covariance_matrix)
    
    # 가장 작은 고유값에 해당하는 고유벡터가 평면의 법선 벡터입니다.
    normal_vector = eigenvectors[:, 0]
    
    # 평면의 방정식: ax + by + cz + d = 0
    # 여기서 (a, b, c)는 법선 벡터를 나타내며, d는 평면과 원점 사이의 거리를 나타냅니다.
    a, b, c = normal_vector
    d = -np.dot(normal_vector, centroid)
    
    # 평면의 파라미터를 반환합니다.
    return a, b, c, d







rospy.init_node('nomal_vector')
posearray = PointsPose(topic="/mouse_click_points")

while not rospy.is_shutdown():
    
    if posearray.point is None:
        pass
    
    else:
        print("running!!")
        points_set = []
        print(len(posearray.point.poses))
        for i in posearray.point.poses:
            points_set.append([i.position.x, i.position.y, i.position.z])


        points = np.array(points_set, dtype=np.float32)

        # 평면 도출
        a, b, c, d = fit_plane_to_points(points)
        print("평면 방정식: {}x + {}y + {}z + {} = 0".format(a, b, c, d))


        # 시각화를 위해 데이터 분리
        x, y, z = points[:, 0], points[:, 1], points[:, 2]

        # 시각화를 위한 메쉬 생성
        xx, yy = np.meshgrid(np.linspace(x.min(), x.max(), 10), np.linspace(y.min(), y.max(), 10))
        zz = -(a * xx + b * yy + d) / c

        # 3D 그래프 생성
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # 데이터 그래프 추가
        ax.scatter(x, y, z, c='b', marker='o', label='Data Points')

        # 평면 그래프 추가
        ax.plot_surface(xx, yy, zz, color='r', alpha=0.5, label='Fitted Plane')

        # 축 라벨 추가
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        
        ax.set_aspect('equal')

        # 범례 표시
        # ax.legend()

        # 그래프 출력
        plt.show()