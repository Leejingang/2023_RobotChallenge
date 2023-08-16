import cv2
import numpy as np
from ketisdk.sensor.realsense_sensor import RSSensor

def remove_blue_and_white(frame):
    # 파란색과 흰색을 제외한 나머지 색상을 제거합니다.
    lower_blue = np.array([0, 0, 50])  # 파란색 범위
    upper_blue = np.array([50, 50, 100])
    lower_white = np.array([200, 200, 200])  # 흰색 범위
    upper_white = np.array([255, 255, 255])

    mask_blue = cv2.inRange(frame, lower_blue, upper_blue)
    mask_white = cv2.inRange(frame, lower_white, upper_white)

    mask = mask_blue | mask_white  # 파란색과 흰색을 합칩니다.

    # 원본 이미지에서 파란색과 흰색을 제외한 부분을 제거합니다.
    result = cv2.bitwise_and(frame, frame, mask=mask)
    return result

if __name__ == "__main__":
    sensor = RSSensor()
    sensor.start()
    
    
    # 카메라로부터 영상을 받아오기 위해 VideoCapture 객체를 생성합니다.
    cap = cv2.VideoCapture(0)  # 0은 기본 카메라를 의미합니다. 만약 여러 개의 카메라가 연결되어있는 경우, 0 대신 다른 값을 사용할 수 있습니다.

    while True:
        # 카메라에서 한 프레임씩 읽어옵니다.
        rgb_data, depth_data = sensor.get_data()
        rgb_data = cv2.cvtColor(rgb_data, cv2.COLOR_BGR2RGB)
        # ret, frame = cap.read()
        # if not ret:
        #     print("카메라로부터 영상을 읽어올 수 없습니다.")
        #     break

        # 파란색과 흰색을 제외한 나머지 부분을 제거합니다.
        processed_frame = remove_blue_and_white(rgb_data)

        # 처리된 프레임을 화면에 출력합니다.
        cv2.imshow("Processed Frame", processed_frame)
        cv2.imshow("original Frame", rgb_data)

        # 'q' 키를 누르면 루프를 종료합니다.
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 사용이 끝난 카메라 객체와 윈도우를 해제합니다.
    cap.release()
    cv2.destroyAllWindows()
