from fastsam import FastSAM, FastSAMPrompt
import torch
import numpy as np
import cv2
import time
from ketisdk.sensor.realsense_sensor import RSSensor
import matplotlib.pyplot as plt
from scipy.stats import linregress
from mpl_toolkits.mplot3d import Axes3D
from sklearn.decomposition import PCA
from sklearn.preprocessing import StandardScaler
import math as m

model = FastSAM("./weights/FastSAM.pt")

DEVICE = torch.device(
    "cuda"
    if torch.cuda.is_available()
    else "mps"
    if torch.backends.mps.is_available()
    else "cpu"
)

print(DEVICE)

sensor = RSSensor()
sensor.start()

class object:
    
    def __init__(self, points):
        self.points = points
        self.center_empty = None
        self.degree = None

def roi_data(data, depth):
    obj_list1 = []
    
    # 각 물체의 x,y 값 저장
    for obj in data:
        
        ob = []
        
        for y in range(len(data[0])):
            for x in range(len(data[0][0])):
                if obj[y][x] != 0:
                    ob.append([x, y])
                    
                else:
                    pass
                
        obj_list1.append(ob)
        
    
    obj_list2 = []
    
    # 물체의 크기가 thre 이상인 물체 제거 : 지면 제거
    for i in obj_list1:
        if len(i) <= 100000:
            obj_list2.append(i)
            
            
    obj_list3 = []
    
    # 뎁스정보 추가 및 뎁스가 0인 이상치 제거
    for j in obj_list2:
        ob2 = []
        
        for k in j:
            t = [k[0], k[1], depth[k[1]][k[0]]]
            if t[2] != 0 :
                ob2.append(t)
            
        obj_list3.append(ob2)
        
    return obj_list2, obj_list3

    # 뎁스정보를 이용해 물체의 평균 높이 계산

def spatial_filltering(data):
    
    i = 0
    obj_list4 = []
        
    for points in data:
        z_values = [point[2] for point in points]
        sorted_z_values = sorted(z_values) #가까운 z축으로 정렬
        top_z_mean = np.mean(np.array(sorted_z_values[:int(len(sorted_z_values) * 0.2)])) #상위 20개의 z축 높이의 평균값 확인
        
        # variance = np.var(z_values)
        # std_deviation = np.std(z_values)
        # # print("number", i)
        # # print("Variance:", variance)
        # # print("Standard Deviation:", std_deviation)
        
        # #물체의 분포도가 존나게 크면 패스
        # if std_deviation > 15:
        #     pass
        
        # #즉당한 분포도면 포함
        # else :
        obj_list4.append([data[i], top_z_mean])
        i += 1

    obj_list4 = sorted(obj_list4, key=lambda x: x[-1]) #z축 높이가 낮은 순서대로 기준으로 정렬 - 높게 위치한 물체 순대로
    
    #가장 높이 위치한 물체의 중심이 비었는지 확인
    for sublist in obj_list4:
        del sublist[-1]
        
    center_point = get_center_point(obj_list4)
    
    
    
    obj_list5 = []
    for  point in obj_list4:
        obj_list5.append(point[0])
    
    return obj_list5


def get_center_point(data):
    
    sum_x = sum(y[0] for y in data)
    sum_y = sum(y[1] for y in data)
    sum_z = sum(y[2] for y in data)

    # 데이터 개수
    num_data = len(data)

    # 중심 계산
    center_x = sum_x / num_data
    center_y = sum_y / num_data    
    
    return (center_x, center_y)
    
def center_empty(depth_img, center_x, center_y, z_value):
        
    radius = 5
    count = 0
    thre = 5
    for x in range(center_x - radius, center_x + radius + 1):
        for y in range(center_y - radius, center_y + radius + 1):
            if z_value - thre < depth_img[y][x] < z_value + thre:
                count += 1
                
    if count >= 10:
        return True
    else:
        return False

def calculate_angle(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    angle = m.degrees(m.atan2(dy, dx))
    return angle
          
    
def get_angle(image, center):
    
    corners = contour(image)
    dis_list = []
    for corner in corners:
        distance = m.sqrt((center[0] - corner[0])**2 + (center[1] - corner[1])**2)
        dis_list.append([corner[0], corner[1], distance])
        
    dis_list = sorted(dis_list, key=lambda x: x[-1]) 
    
    angle = calculate_angle(center[0], center[1], dis_list[-1][0], dis_list[-1][1])
    
    return angle

def contour(image):
    
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    image = cv2.GaussianBlur(image, (5, 5), 0)
    # Shi-Tomasi 코너 감지 파라미터 설정
    max_corners = 200  # 찾을 최대 코너 개수
    quality_level = 0.01  # 코너로 간주되는 최소한의 품질
    min_distance = 10  # 코너 사이의 최소 거리

    # Shi-Tomasi 코너 감지 수행
    corners = cv2.goodFeaturesToTrack(image, max_corners, quality_level, min_distance)

    # 감지된 코너 좌표 정수로 변환
    corners = np.int32(corners)

    # 감지된 코너에 원 그리기
    corner_list = []
    
    for corner in corners:
        x, y = corner.ravel()
        corner_list.append([x,y])
        cv2.circle(image, (x, y), 3, 255, -1)

    return corner_list



def fit_plane_to_points(points):

    # 데이터를 표준화 (Standardization)
    scaler = StandardScaler()
    points_scaled = scaler.fit_transform(points)

    # PCA를 사용하여 데이터 차원을 2차원으로 축소
    pca = PCA(n_components=2)
    principal_components = pca.fit_transform(points_scaled)

    # PCA로 축소된 데이터로부터 평면 방정식 도출
    normal_vector = pca.components_[0]
    bias = -np.dot(normal_vector, pca.mean_)

    # 평면 방정식 출력
    print("평면 방정식:")
    print(f"{normal_vector[0]} * x + {normal_vector[1]} * y + {normal_vector[2]} * z + {bias} = 0")

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 원본 데이터 포인트
    ax.scatter([point[0] for point in points], [point[1] for point in points], [point[2] for point in points], c='b', marker='o', label='Data Points')

    # 근사화된 평면의 법선 벡터와 평면 위의 점 생성
    xx, yy = np.meshgrid(np.linspace(min([point[0] for point in points]), max([point[0] for point in points]), num=10),
                        np.linspace(min([point[1] for point in points]), max([point[1] for point in points]), num=10))
    zz = (-normal_vector[0] * xx - normal_vector[1] * yy - bias) / normal_vector[2]

    # 근사화된 평면 시각화
    ax.plot_surface(xx, yy, zz, color='r', alpha=0.3, label='Approximating Plane')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Approximating Plane and Data Points')
    # ax.legend(loc='upper right')
    plt.show()



def trans_3d(array, depth_data, ppx, ppy, fx, fy):

    Xc = array[0] 
    Yc = array[1]
    Zc = depth_data[Yc][Xc]
    point_x = ((Xc-ppx)/fx*Zc)
    point_y = ((Yc-ppy)/fy*Zc)
    point_z = float(Zc)
    
    # return [point_x, point_y, point_z]
    return [point_x, point_y, point_z]




# while True:
    
rgb_data = cv2.imread('/home/lee/fastsam/FastSAM/images/0_img.png')
depth_data = cv2.imread('/home/lee/fastsam/FastSAM/images/0_depth.png', -1)
ppx, ppy, fx, fy = np.load("/home/lee/fastsam/FastSAM/images/intr_param.npy")
    
# ROI를 형성하는 네 개의 좌표
vertices = np.array([(488, 20), (497, 678), (908, 682), (907, 19)])
# 검은 배경 생성
black_background = np.zeros_like(rgb_data)
# ROI 영역을 검은 배경에 삽입
cv2.fillPoly(black_background, [vertices], (255, 255, 255))  # 다각형 내부를 흰색으로 채움

# 검은 배경과 원본 이미지를 합치기
result = cv2.bitwise_and(rgb_data, black_background)
    
rgb_copy = result.copy()

everything_results = model(
    source=rgb_copy,
    device=DEVICE,
    retina_masks=True,
    imgsz=1024,
    conf=0.4,
    iou=0.9,
)

prompt_process = FastSAMPrompt(rgb_copy, everything_results, device=DEVICE)
ann = prompt_process.everything_prompt()
img = prompt_process.plot_to_result(annotations=ann)

points = roi_data(prompt_process.results[0].masks.data.tolist(), depth_data)
# points = spatial_filltering(points)




aa = np.zeros((720, 1280, 3), dtype=np.uint8)
for coords in points[:3]:
    for coord in coords:
        cv2.circle(aa, (coord[0], coord[1]), 1, (0, 255, 0), -1)
cv2.imshow('Image with Coordinates', aa)


contou = contour(aa)
cv2.imshow('Image tt', contou)


p = 0
for point1 in points[:3]:
    for point2 in point1:
        cv2.circle(result, (point2[0], point2[1]), 1, (0, 0, p), -1)
    p += 2

# cv2.imshow('img', img)
# cv2.imshow('result', result)
# cv2.imshow('original', rgb_data)

cv2.waitKey(0)
cv2.destroyAllWindows()

# if cv2.waitKey(1) & 0xFF == ord('q'):
#     cv2.destroyAllWindows()

    # break
    
# cv2.destroyAllWindows()
