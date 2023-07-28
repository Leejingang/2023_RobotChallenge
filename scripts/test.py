import numpy as np
import matplotlib.pyplot as plt

def distance_between_points(pt1, pt2):
    x1, y1 = pt1
    x2, y2 = pt2

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

# 배열의 크기 정의
array_width = 640
array_height = 480

points = []

for i in range(array_width):
    for j in range(array_height):
        points.append([i, j])

# 선분 정의 (예제용)
pt1 = (100, 100)
pt2 = (500, 300)

# 선분 근처의 점들 구하기 (threshold 값에 따라 거리 범위를 조절)
threshold = 5
points_near_line = get_points_near_line(points, pt1, pt2, threshold)
print("선분 근처의 점들:", points_near_line)

# 시각화
x, y = zip(*points)
x_near, y_near = zip(*points_near_line)

plt.plot(x, y, 'bo', label='All Points')
plt.plot(x_near, y_near, 'ro', label='Points Near Line')
plt.plot([pt1[0], pt2[0]], [pt1[1], pt2[1]], 'g-', label='Line')

plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('Points Near Line')
plt.legend()
plt.grid(True)
plt.show()
