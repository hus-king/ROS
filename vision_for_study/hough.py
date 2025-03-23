import cv2
import numpy as np

def enhance_circle_edges(image):
    """增强圆形边缘"""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # 高斯模糊降噪
    blurred = cv2.GaussianBlur(gray, (7, 7), 2)
    # 直方图均衡化
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    enhanced = clahe.apply(blurred)
    return enhanced

def detect_circles(frame):
    """核心检测逻辑"""
    # 预处理
    edge_enhanced = enhance_circle_edges(frame)
    
    # 霍夫圆检测参数
    circles = cv2.HoughCircles(
        edge_enhanced,
        cv2.HOUGH_GRADIENT,
        dp=1,              # 保持原始分辨率
        minDist=50,        # 圆心最小间距（像素）
        param1=120,        # Canny高阈值（较高值减少边缘干扰）
        param2=70,         # 累加器阈值（较高值减少误检）
        minRadius=30,      # 最小半径
        maxRadius=250      # 最大半径
    )
    
    # 结果处理
    valid_circles = []
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for circle in circles[0, :]:
            x, y, r = circle[0], circle[1], circle[2]
            valid_circles.append((x, y, r))
    return valid_circles

# ---------------------- 主程序 ----------------------
if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 检测圆形
        circles = detect_circles(frame)

        # 绘制结果（单圆模式）
        for x, y, r in circles:
            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)  # 绿色圆
            cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)  # 红色圆心

        cv2.imshow("Circle Detection", frame)
        if cv2.waitKey(1) == 27:
            break

    cap.release()
    cv2.destroyAllWindows()