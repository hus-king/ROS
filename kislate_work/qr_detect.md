###### pyzbar
- `pyzbar`包中有`decode`方法,用于对二维码的解码
- 可以用`from pyzbar.pyzbar import decode`引入
- 该函数返回元素为`Decoded`对象的一个列表(如decode_objects = decode(cv_image))
- 每个对象包含以下属性
```
1.data:解码后的数据,通常为一个字符对象,可以用decode方法转换为字符串:obj.data.decode("utf-8")
2.type:条形码或者二维码类型
3.rect:条形码或二维码在图像中的位置:(left,top,width,height)
4.polygon:其轮廓,一个包含多个顶点的多边形?
```
下面尝试使用`pyzbar`来实现二维码检测,因底层依赖`zbar`配置失败而作罢,代码如下:
```python
import cv2
from pyzbar.pyzbar import decode

def decode_qr_code(frame):
    """
    Decode QR codes in the image
    :param frame: Current frame image
    :return: Image with QR code content
    """
    decoded_objects = decode(frame)
    if decoded_objects:
        for obj in decoded_objects:
            # Get QR code content and type
            qr_data = obj.data.decode("utf-8")
            qr_type = obj.type

            # Draw the bounding box of the QR code
            rect = obj.rect
            cv2.rectangle(frame, (rect.left, rect.top), (rect.left + rect.width, rect.top + rect.height), (0, 255, 0), 2)
            cv2.putText(frame, f"{qr_type}: {qr_data}", (rect.left, rect.top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            print(f"QR Code Type: {qr_type}")
            print(f"QR Code Content: {qr_data}")
    return frame

def main():
    # Open the camera
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Unable to open the camera!")
        return

    print("Press 'q' to quit the program...")
    while True:
        # Read each frame from the camera
        ret, frame = cap.read()
        if not ret:
            print("Unable to read camera frame!")
            break

        # Decode QR codes
        frame = decode_qr_code(frame)

        # Display the image
        cv2.imshow("QR Code Scanner", frame)

        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release camera resources and close windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
```
可以直接使用`cv2`中对于二维码的识别模块,不能检测条形码
```python
import cv2

def real_time_qr_code_detection():
    cap = cv2.VideoCapture(0)  # capture video from camera 0
    detector = cv2.QRCodeDetector()

    print("Press 'q' to quit")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Unable to capture video")
            break


        data, bbox, _ = detector.detectAndDecode(frame)
        if data:
            print("QR Code detected: ", data)
            
            if bbox is not None:
                bbox = bbox.astype(int)
                cv2.polylines(frame, [bbox], True, (0, 255, 0), 2)
                
                
                top_left_corner = tuple(bbox[0][0]) 
                cv2.putText(frame, data, top_left_corner, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
        # show the frame
        cv2.imshow("QR Code Scanner", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    real_time_qr_code_detection()
```
显示测试如下,可以手动尝试扫描一下图二有惊喜(不是

![测试图片1](images/qr_hello.png)
![测试图片2](images/qr_mine.png)
