import rospy
from std_msgs.msg import String  # 예제로 사용할 메시지 타입 (원하는 메시지 타입으로 변경)

def callback(data):
    # 토픽 메시지가 도착했을 때 실행될 콜백 함수
    rospy.loginfo("Received message: %s", data.data)

def listener():
    # 노드 초기화
    rospy.init_node('listener', anonymous=True)

    # 토픽 구독자 생성
    rospy.Subscriber('your_topic', String, callback)  # 구독하고자 하는 토픽과 메시지 타입

    # 반복 주기를 설정 (1초당 1회)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # 반복적으로 루프를 돌면서 토픽 수신
        rate.sleep()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
