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

class grasp:
    
    def __init__(self, point, center, angle):
        self.point = point
        self.center_empty = center
        self.angle = angle
        
def roi_data(data, depth):
    obj_list1 = []
    for obj in data:
        ob = []
        for y in range(len(data[0])):
            for x in range(len(data[0][0])):
                if obj[y][x] != 0:
                    ob.append([x, y])    
                else:
                    pass
        
        if len(ob) <= 100000:
            obj_list1.append(ob)
            
                    
    obj_list2 = []
    obj_list3 = []
    for j in obj_list1:
        ob1 = []
        ob2 = []
        for k in j:
            t = [k[0], k[1], depth[k[1]][k[0]]]
            ob1.append(t)
            if t[2] != 0 :
                ob2.append(t)
        obj_list2.append(ob1)
        obj_list3.append(ob2)
        
    return obj_list2, obj_list3


def spatial_filltering(points1, points2):
    
    i = 0
    
    obj_list4 = []    
    obj_list5 = []
    
    for points in points2:
        z_values = [point[2] for point in points]
        sorted_z_values = sorted(z_values)
        top_z_mean = np.mean(np.array(sorted_z_values[:int(len(sorted_z_values) * 0.4)]))
        obj_list4.append([points2[i], top_z_mean])
        obj_list5.append([points1[i], top_z_mean])
        i += 1

    obj_list4 = sorted(obj_list4, key=lambda x: x[-1])
    obj_list5 = sorted(obj_list5, key=lambda x: x[-1])


    number = 0
    
    z_mean = obj_list4[number][-1]
    
    for sublist in obj_list4:
        del sublist[-1]
        
    for sublist in obj_list5:
        del sublist[-1]
        
    center_point = get_center_point(obj_list5[number][0])
    
    
    # obj_list5 = []
    # for  point in obj_list4:
    #     obj_list5.append(point[0])
    
    
    return obj_list5[number][0], center_point, z_mean


def get_center_point(data):
    
    sum_x = sum(y[0] for y in data)
    sum_y = sum(y[1] for y in data)

    num_data = len(data)

    center_x = sum_x / num_data
    center_y = sum_y / num_data    
    
    return (int(center_x), int(center_y))
    
def center_empty(depth_img, center_x, center_y, z_value):
        
    radius = 5
    count = 0
    thre = 5
    for x in range(center_x - radius, center_x + radius + 1):
        for y in range(center_y - radius, center_y + radius + 1):
            if z_value - thre < depth_img[y][x] < z_value + thre:
                count += 1
                
    if count >= 10: #채워져 있는 갯수가 많으면 -> 사각형 정면이 아니면
        return False
    else:
        return True

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
    
    return angle, [dis_list[-1][0], dis_list[-1][1]]

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
        # cv2.circle(image, (x, y), 3, 255, -1)

    return corner_list

def trans_3d(array, depth_data, ppx, ppy, fx, fy):

    Xc = array[0] 
    Yc = array[1]
    Zc = depth_data[Yc][Xc]
    point_x = ((Xc-ppx)/fx*Zc)
    point_y = ((Yc-ppy)/fy*Zc)
    point_z = float(Zc)
    
    return [point_x, point_y, point_z]


def get_target_img(points):
    zero_img = np.zeros((720, 1280, 3), dtype=np.uint8)
    for coords in points:
        cv2.circle(zero_img, (coords[0], coords[1]), 1, (0, 255, 0), -1)
    
    return zero_img
    






model = FastSAM("./weights/FastSAM.pt")

rgb_data = cv2.imread('/home/lee/fastsam/FastSAM/images/0_img.png')
depth_data = cv2.imread('/home/lee/fastsam/FastSAM/images/0_depth.png', -1)
ppx, ppy, fx, fy = np.load("/home/lee/fastsam/FastSAM/images/intr_param.npy")

vertices = np.array([(488, 20), (497, 678), (908, 682), (907, 19)])
black_background = np.zeros_like(rgb_data)
cv2.fillPoly(black_background, [vertices], (255, 255, 255))  # 다각형 내부를 흰색으로 채움
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

points1, points2 = roi_data(prompt_process.results[0].masks.data.tolist(), depth_data)

target_object, center, z_mean = spatial_filltering(points1, points2)

empty = center_empty(depth_data, int(center[0]), int(center[1]), z_mean)
target_img = get_target_img(target_object)

angle, far_point = get_angle(target_img, center)

cv2.imshow("target", target_img)
for point in target_object:
    cv2.circle(result, (point[0], point[1]), 1, (0,255,0), -1)
    
cv2.circle(result, far_point, 5, (0,0,255),  -1)
cv2.circle(result, center, 5, (255,0,0),  -1)
cv2.circle(img, center, 5, (255,0,0),  -1)


cv2.imshow('img', img)
cv2.imshow('result', result)
# cv2.imshow('original', rgb_data)

cv2.waitKey(0)
cv2.destroyAllWindows()

