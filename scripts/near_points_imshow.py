import cv2
import rospy
from geometry_msgs.msg import PoseArray
import numpy as np

def points_callback(msg):
    # ROS 메시지에서 점들 추출
    points = []
    for pose in msg.poses:
        points.append((int(pose.position.x), int(pose.position.y)))

    # OpenCV 창 생성
    img_width, img_height = 1280, 720
    img = np.zeros((img_height, img_width, 3), dtype=np.uint8)

    # 점들을 OpenCV 창에 그리기
    for point in points:
        print(point)
        cv2.circle(img, point, 5, (0, 0, 255), -1)

    # OpenCV 창 열기
    cv2.imshow("Points", img)
    cv2.waitKey(0)

def main():
    # ROS 노드 초기화
    rospy.init_node('point_subscriber', anonymous=True)

    # ROS 메시지 구독자 생성
    rospy.Subscriber('mouse_click_points', PoseArray, points_callback)

    # ROS 메시지를 받기 위해 무한 루프로 진입
    rospy.spin()

if __name__ == '__main__':
    main()
