import cv2
import rospy
from geometry_msgs.msg import Point, PoseArray, Pose
from std_msgs.msg import Header
import numpy as np
import matplotlib.pyplot as plt

from ketisdk.sensor.realsense_sensor import RSSensor


def trans_3d(array, depth_data, intr_params):
    #depth_data : rgb_data, depth_data = sensor.get_data()
    # sensor = RSSensor()
    # sensor.start()
    # ppx, ppy, fx, fy = sensor.intr_params.ppx, sensor.intr_params.ppy, sensor.intr_params.fx, sensor.intr_params.fy
    Xc = array[0] 
    Yc = array[1]
    Zc = depth_data[Yc,Xc]
    point_x = ((Xc-intr_params[0])/intr_params[2]*Zc)
    point_y = ((Yc-intr_params[1])/intr_params[3]*Zc)
    point_z = float(Zc)
    
    # return [point_x, point_y, point_z]
    return [Xc, Yc, Zc]

def distance_between_points(pt1, pt2):
    x1, y1 = pt1[0], pt1[1]
    x2, y2 = pt2[0], pt2[1]

    # 두 점 사이의 거리 계산
    distance = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    return distance

def get_points_near_line(points, line_pt1, line_pt2, threshold, depth_data):
    points_near_line = []

    for pt in points:
        distance = distance_between_points(pt, line_pt1) + distance_between_points(pt, line_pt2)

        if abs(distance - distance_between_points(line_pt1, line_pt2)) <= threshold and max(depth_data[line_pt1[1]][line_pt1[0]], depth_data[line_pt2[1]][line_pt2[0]]) + 1 > depth_data[pt[1], pt[0]] > min(depth_data[line_pt1[1]][line_pt1[0]], depth_data[line_pt2[1]][line_pt2[0]]) - 1:
            points_near_line.append(pt)

    return points_near_line



def get_mouse_click():

    def mouse_callback(event, x, y, flags, param):
        
        nonlocal click_count, click_points, publisher

        if event == cv2.EVENT_LBUTTONDOWN:
            click_points.append((x, y))
            click_count += 1

            if click_count == 2:
                print("click!!")
                x1, y1 = click_points[0]
                x2, y2 = click_points[1]                
                center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                

                point_1 = trans_3d([x1, y1], depth_data, intr_params)
                point_2 = trans_3d([x2, y2], depth_data, intr_params)
                
                print(point_1)
                print(point_2)
                # print('point', point_1, point_2)
                point_center = trans_3d([center_x, center_y], depth_data, intr_params)
                points_near_line = get_points_near_line(temp_img, (point_1[0], point_1[1]), (point_2[0], point_2[1]), 2, depth_data)
                points_near_line2 = []
                points_near_line2 = points_near_line
                for k in range(len(points_near_line2)):
                    points_near_line2[k].append(depth_data[int(points_near_line[k][1])][int(points_near_line[k][0])])
                    
                print(len(points_near_line))
                # 메시지 생성
                header = Header()
                header.stamp = rospy.Time.now()
                header.frame_id = 'camera'

                point_msg = PoseArray()
                point_msg.header = header
                point_msg.poses = []
                
                
                print(points_near_line2)
                
                
                for i in range(len(points_near_line)):
                    po = Pose()
                    po.position.x = points_near_line[i][0]
                    po.position.y = points_near_line[i][1]
                    po.position.z = depth_data[int(points_near_line[i][1])][int(points_near_line[i][0])]
                    
                    point_msg.poses.append(po)
                    
                publisher.publish(point_msg)
                # print(point_msg.poses)

                # 클릭 정보 초기화
                click_count = 0
                click_points.clear()
            
                

                # x, y = zip(*temp_img)
                # x_near, y_near = zip(*points_near_line)


    # ROS 노드 초기화
    rospy.init_node('mouse_click_publisher', anonymous=True)
    publisher = rospy.Publisher('mouse_click_points', PoseArray, queue_size=1)

    # # 비디오 캡처 객체 생성
    # cap = cv2.VideoCapture(0)  # 카메라 번호 (일반적으로 0번: 내장 웹캠)

    # # 캡처 성공 여부 확인
    # if not cap.isOpened():
    #     print("카메라를 열 수 없습니다.")
    #     return

    # # 마우스 이벤트 콜백 함수 등록
    cv2.namedWindow("Camera")
    cv2.setMouseCallback("Camera", mouse_callback)


    click_count = 0
    click_points = []

    while True:
    #     ret, frame = cap.read()

    #     if not ret:
    #         print("비디오를 읽을 수 없습니다.")
    #         break
    
        rgb_data, depth_data = sensor.get_data()

        flaten_depth_data = depth_data.flatten()
        # rgb_data = cv2.imread("/home/lee/Desktop/robotchallenge/robochalle_demo/data/Grasp_RGB.png")
        rgb_data_with_points = rgb_data.copy()

        # points_near_line에 있는 점들을 빨간색으로 표시

        cv2.imshow("Camera", rgb_data)
        # cv2.imshow("depth", depth_data)

        # print("rgb and depth size : ", rgb_data.shape, "  ", depth_data.shape)
        # 키 입력 대기 (1ms 동안)
        key = cv2.waitKey(1) & 0xFF

        # 'q' 키를 누르면 루프 종료
        if key == ord('q'):
            break

    # cap.release()
    # cv2.destroyAllWindows()

# 무한 루프를 사용하여 클릭을 두 번 할 때마다 두 좌표와 중앙값을 ROS로 publish

sensor = RSSensor()
sensor.start()


global intr_params, rgb_data, depth_data, flaten_depth_data, temp_img



# ppx, ppy, fx, fy = np.load("/home/lee/Desktop/robotchallenge/robochalle_demo/APPS/intr_param.npy")
intr_params = [sensor.intr_params.ppx, sensor.intr_params.ppy, sensor.intr_params.fx, sensor.intr_params.fy]


# intr_params = [ppx, ppy, fx, fy]

# rgb_data = cv2.imread("/home/lee/Desktop/robotchallenge/robochalle_demo/data/Grasp_RGB.png")
# depth_data = cv2.imread("/home/lee/Desktop/robotchallenge/robochalle_demo/data/Grasp_depth.png", -1)

# height, width, channels = rgb_data.shape
temp_img = []
for i in range(1280):
    for j in range(720):
        temp_img.append([i, j])                

while not rospy.is_shutdown():
    get_mouse_click()