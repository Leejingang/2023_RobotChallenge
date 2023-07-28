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

def get_points_near_line(points, line_pt1, line_pt2, threshold):
    points_near_line = []

    for pt in points:
        distance = distance_between_points(pt, line_pt1) + distance_between_points(pt, line_pt2)

        if abs(distance - distance_between_points(line_pt1, line_pt2)) <= threshold:
            points_near_line.append(pt)

    return points_near_line



def get_mouse_click():

    def mouse_callback(event, x, y, flags, param):
        
        # rgb_data, depth_data = sensor.get_data()
        


        
        
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
                
                points_near_line = get_points_near_line(temp_img, (point_1[0], point_1[1]), (point_2[0], point_2[1]), 2)
                
                print(len(points_near_line))
                
                # 메시지 생성
                header = Header()
                header.stamp = rospy.Time.now()
                header.frame_id = 'camera'

                point_msg = PoseArray()
                point_msg.header = header
                point_msg.poses = []
                
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
                
                
                x, y = zip(*temp_img)
                x_near, y_near = zip(*points_near_line)

                # plt.plot(x, y, 'bo', label='All Points')
                # plt.plot(x_near, y_near, 'ro', label='Points Near Line')
                # plt.plot([point_1[0], point_2[0]], [point_1[1], point_2[1]], 'g-', label='Line')

                # plt.xlabel('X-axis')
                # plt.ylabel('Y-axis')
                # plt.title('Points Near Line')
                # plt.legend()
                # plt.grid(True)
                # plt.show()
                # plt.draw()
                # plt.pause(2)


    # ROS 노드 초기화
    rospy.init_node('mouse_click_publisher', anonymous=True)
    publisher = rospy.Publisher('mouse_click_points', PoseArray, queue_size=10)

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
        rgb_data = cv2.imread("/home/lee/Desktop/robotchallenge/robochalle_demo/data/Grasp_RGB.png")

        cv2.imshow("Camera", rgb_data)

        # 키 입력 대기 (1ms 동안)
        key = cv2.waitKey(1) & 0xFF

        # 'q' 키를 누르면 루프 종료
        if key == ord('q'):
            break

    # cap.release()
    # cv2.destroyAllWindows()

# 무한 루프를 사용하여 클릭을 두 번 할 때마다 두 좌표와 중앙값을 ROS로 publish

# sensor = RSSensor()
# sensor.start()

ppx, ppy, fx, fy = np.load("/home/lee/Desktop/robotchallenge/robochalle_demo/APPS/intr_param.npy")
# intr_params = [sensor.intr_params.ppx, sensor.intr_params.ppy, sensor.intr_params.fx, sensor.intr_params.fy]

global intr_params, rgb_data, depth_data, flaten_depth_data, temp_img

intr_params = [ppx, ppy, fx, fy]

rgb_data = cv2.imread("/home/lee/Desktop/robotchallenge/robochalle_demo/data/Grasp_RGB.png")
depth_data = cv2.imread("/home/lee/Desktop/robotchallenge/robochalle_demo/data/Grasp_depth.png", -1)

flaten_depth_data = depth_data.flatten()
height, width, channels = rgb_data.shape
temp_img = []
for i in range(width):
    for j in range(height):
        temp_img.append([i, j])                

while not rospy.is_shutdown():
    get_mouse_click()
