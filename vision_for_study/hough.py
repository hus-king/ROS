import cv2  # OpenCV库
import numpy as np  # NumPy库

def enhance_circle_edges(image):  # 增强圆形边缘
    """增强圆形边缘
    :param image: 输入的彩色图像
    :return: 增强后的灰度图像
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  # 转换为灰度图像
    # 高斯模糊降噪
    blurred = cv2.GaussianBlur(gray, (7, 7), 2)  # 应用高斯模糊, (7, 7)为内核大小, 2为标准差
    # 直方图均衡化
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))  # 创建CLAHE对象, clipLimit为对比度限制, tileGridSize为网格大小
    enhanced = clahe.apply(blurred)  # 应用CLAHE
    return enhanced  # 返回增强后的图像

def detect_circles(frame):  # 检测圆形
    """核心检测逻辑
    :param frame: 输入的彩色图像帧
    :return: 检测到的圆形列表，每个圆形以(x, y, r)表示
    """
    # 预处理
    edge_enhanced = enhance_circle_edges(frame)  # 增强边缘
    
    # 霍夫圆检测参数
    circles = cv2.HoughCircles(
        edge_enhanced,  # 输入图像
        cv2.HOUGH_GRADIENT,  # 检测方法
        dp=1,  # 保持原始分辨率
        minDist=50,  # 圆心最小间距（像素）
        param1=120,  # Canny高阈值（较高值减少边缘干扰）
        param2=70,  # 累加器阈值（较高值减少误检）
        minRadius=30,  # 最小半径
        maxRadius=250  # 最大半径
    )
    
    # 结果处理
    valid_circles = []  # 有效圆形列表
    if circles is not None:  # 如果检测到圆形
        circles = np.uint16(np.around(circles))  # 四舍五入并转换为无符号整数
        for circle in circles[0, :]:  # 遍历每个圆形
            x, y, r = circle[0], circle[1], circle[2]  # 获取圆心坐标和半径
            valid_circles.append((x, y, r))  # 添加到有效圆形列表
    return valid_circles  # 返回有效圆形列表

# ---------------------- 主程序 ----------------------
if __name__ == "__main__":  # 主程序入口
    cap = cv2.VideoCapture(0)  # 打开摄像头
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)  # 设置帧宽度
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)  # 设置帧高度

    while True:  # 循环读取帧
        ret, frame = cap.read()  # 读取一帧
        if not ret:  # 如果读取失败
            break  # 退出循环

        # 检测圆形
        circles = detect_circles(frame)  # 检测圆形

        # 绘制结果（单圆模式）
        for x, y, r in circles:  # 遍历每个圆形
            # 绘制圆形: 圆心、半径、颜色、线宽
            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)  # 绘制绿色圆
            cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)  # 绘制红色圆心

        cv2.imshow("Circle Detection", frame)  # 显示结果
        if cv2.waitKey(1) == 27:  # 按下ESC键退出
            break  # 退出循环

    cap.release()  # 释放摄像头
    cv2.destroyAllWindows()  # 关闭所有窗口