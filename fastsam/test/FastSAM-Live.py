from fastsam import FastSAM, FastSAMPrompt
import torch
import numpy as np
import cv2
import time
from ketisdk.sensor.realsense_sensor import RSSensor
import matplotlib.pyplot as plt
from scipy.stats import linregress
from mpl_toolkits.mplot3d import Axes3D

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



def roi_data(data, depth):
    print(len(data))
    obj_list1 = []
    
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
    
    for i in obj_list1:
        if len(i) <= 10000:
            obj_list2.append(i)
            
            
    obj_list3 = []
    
    for j in obj_list2:
        ob2 = []
        
        for k in j:
            t = [k[0], k[1], depth[k[1]][k[0]]]
            if t[2] != 0 :
                ob2.append(t)
            
        obj_list3.append(ob2)
        
    return obj_list3

def spatial_filltering(data):
    i = 0
    
    obj_list4 = []
    
    for points in data:
        z_values = [point[2] for point in points]
        variance = np.var(z_values)
        std_deviation = np.std(z_values)
        print("number", i)
        print("Variance:", variance)
        print("Standard Deviation:", std_deviation)
        
        if std_deviation > 50:
            pass
        
        else :
            obj_list4.append(data[i])
        i += 1

    return obj_list4




def fit_plane_to_points(points):



    # 좌표를 각 축에 대한 리스트로 변환
    x_values = [x for x, _, _ in points]
    y_values = [y for _, y, _ in points]
    z_values = [z for _, _, z in points]

    # linregress 함수를 사용하여 최소 제곱법으로 평면의 방정식 계산
    slope_x, intercept_x, r_value_x, p_value_x, std_err_x = linregress(x_values, z_values)
    slope_y, intercept_y, r_value_y, p_value_y, std_err_y = linregress(y_values, z_values)

    # 평면의 방정식: {}x + {}y + {}z = {}
    a = slope_x
    b = slope_y
    c = 0.5  # 계수 c는 직접 설정하여야 합니다.
    d = intercept_x + intercept_y

    print("평면의 방정식: {}x + {}y + {}z = {}".format(a, b, c, d))
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 3차원 좌표들을 점으로 표시
    ax.scatter(x_values, y_values, z_values, c='b', marker='o')

    # 평면의 방정식을 시각화
    xx, yy = np.meshgrid(np.arange(min(x_values), max(x_values), 10),
                        np.arange(min(y_values), max(y_values), 10))
    zz = a*xx + b*yy + c
    ax.plot_surface(xx, yy, zz, color='r', alpha=0.3)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_box_aspect([1,1,1])
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




while True:

    rgb_data, depth_data = sensor.get_data()
    print(len(depth_data))
    print(len(depth_data[0]))
    ppx, ppy, fx, fy = sensor.intr_params.ppx, sensor.intr_params.ppy, sensor.intr_params.fx, sensor.intr_params.fy

    start = time.perf_counter()
    

    # ROI를 형성하는 네 개의 좌표
    vertices = np.array([[881, 28], [499, 34], [505, 681], [890, 666]])

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
    
    # print(everything_results[0].masks.data[0].tolist())
    
    # cv2.imshow("masking", everything_results[0].masks[0].tolist())
    
    end = time.perf_counter()
    total_time = end - start
    fps = 1 / total_time
    
    prompt_process = FastSAMPrompt(rgb_copy, everything_results, device=DEVICE)
    
    # everything prompt
    ann = prompt_process.everything_prompt()
    img = prompt_process.plot_to_result(annotations=ann)
    
    
    print(len(depth_data[0]))
    points = roi_data(prompt_process.results[0].masks.data.tolist(), depth_data)
    # points = spatial_filltering(points)
    
    new_points = []
    for i in points[0]:
        poi = trans_3d(i, depth_data, ppx, ppy, fx, fy)
        new_points.append(poi)
        
    fit_plane_to_points(new_points)
    
    
    
    

    
    for point1 in points[0]:
        for point2 in point1:
            cv2.circle(img, (point2[0], point2[1]), 1, (0, 255, 0), -1)
    
    time.sleep(3)

    cv2.imshow('frame', rgb_copy)
    cv2.imshow('img', img)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
cv2.destroyAllWindows()
