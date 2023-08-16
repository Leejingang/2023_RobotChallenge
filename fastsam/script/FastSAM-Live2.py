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
        
    # fit_plane_to_points(new_points)
    
    
    
    

    
    for point1 in points:
        for point2 in point1:
            cv2.circle(img, (point2[0], point2[1]), 1, (0, 255, 0), -1)
    
    time.sleep(3)

    cv2.imshow('frame', rgb_copy)
    cv2.imshow('img', img)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
cv2.destroyAllWindows()
