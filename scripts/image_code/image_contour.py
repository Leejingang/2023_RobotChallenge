import cv2
import numpy as np
import cv2
import rospy
from geometry_msgs.msg import Point, PoseArray, Pose
from std_msgs.msg import Header
import numpy as np
import matplotlib.pyplot as plt
import time
from ketisdk.sensor.realsense_sensor import RSSensor


def find_contours(image):
    # 이미지 불러오기
    image = cv2.UMat(image)

    # 이미지를 그레이스케일로 변환
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # 가장자리 검출 (Canny edge detection)
    edges = cv2.Canny(gray, 10, 150)

    # 컨투어 찾기
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 컨투어를 그리는 색상 설정 (빨간색)
    color = (0, 0, 255)

    print(contours[0].get())
    # 모든 컨투어 그리기
    for contour in contours:
        print(contour.get())
        cv2.drawContours(image, [contour], -1, color, 2)

    # 이미지에 컨투어가 그려진 결과 보여주기
    cv2.imshow("Contours", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    cv2.namedWindow('result')

    sensor = RSSensor()
    sensor.start()
    
    rgb_data, depth_data = sensor.get_data()

    # image_path = "path/to/your/image.jpg"  # 이미지 파일 경로를 적절히 입력하세요
    find_contours(rgb_data)
