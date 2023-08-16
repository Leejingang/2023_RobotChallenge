import cv2
import numpy as np

# 이미지 로드
image = cv2.imread('data/0_img.png', cv2.IMREAD_GRAYSCALE)

# 에지 검출
edges = cv2.Canny(image, threshold1=50, threshold2=150)

# 컨투어 검출
contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# 분류 작업 수행
classified_shapes = []
for contour in contours:
    perimeter = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
    
    if len(approx) == 3:
        shape = "Triangle"
    elif len(approx) == 4:
        shape = "Rectangle"
    else:
        shape = "Unknown"
    
    classified_shapes.append((shape, contour))

# 결과 출력
result_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
for shape, contour in classified_shapes:
    cv2.drawContours(result_image, [contour], -1, (0, 255, 0), 2)
    cv2.putText(result_image, shape, (contour[0][0][0], contour[0][0][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

cv2.imshow('Classified Shapes', result_image)
cv2.waitKey(0)
cv2.destroyAllWindows()