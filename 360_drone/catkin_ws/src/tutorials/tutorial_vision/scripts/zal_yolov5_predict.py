#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os 
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pathlib import Path
from tutorial_vision.msg import BoundingBox, BoundingBoxes
import torch
import torch.nn.functional as F

ROOT = ''   
if __name__ == '__main__':
    rospy.init_node('zal_yolov5_predict_node', sys.argv)
    ROOT = rospy.get_param('~yolov5_root')
else:
    FILE = Path(__file__).resolve()
    ROOT = FILE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))

from models.common import DetectMultiBackend
from utils.augmentations import classify_transforms
from utils.general import LOGGER, check_requirements, colorstr, increment_path, print_args, set_logging, non_max_suppression
from utils.torch_utils import select_device, smart_inference_mode, time_sync


@smart_inference_mode()
def detect(
    weights,  # weight path
    source,  # image
    imgsz=224,  # inference size
    device='',  # cuda device No. or cpu
    half=False,  # use FP15 half-precision inference
    dnn=False,  # use OpenCV DNN for ONNX inference
    confidence_threshold=0.85  # confidence threshold
):
    file = str(source)
    seen, dt = 1, [0.0, 0.0, 0.0]
    device = select_device(device)
    
    # Transforms
    transforms = classify_transforms(imgsz)

    # Load model
    model = DetectMultiBackend(weights, device=device, dnn=dnn, fp16=half)
    names = model.names
    model.warmup(imgsz=(1, 3, imgsz, imgsz))  # warmup # DEBUG only

    # Image
    t1 = time_sync()
    im = cv2.cvtColor(source, cv2.COLOR_BGR2RGB)
    im = torch.from_numpy(im).permute(2, 0, 1).to(device)  # rearrange dimensions
    im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
    im /= 255  # 0 - 255 to 0.0 - 1.0
    if len(im.shape) == 3:
        im = im[None]  # expand for batch dim
    t2 = time_sync()
    dt[0] += t2 - t1

    # Inference
    pred = model(im)
    t3 = time_sync()
    dt[1] += t3 - t2
    
    boxes = []
    
    pred = non_max_suppression(pred, confidence_threshold, iou_thres=0.45)
    # Process predictions
    for det in pred:
        if len(det):
            for *xyxy, conf, cls in reversed(det):
                box = BoundingBox()
                xmin, ymin, xmax, ymax = xyxy
                box.xmin, box.ymin, box.xmax, box.ymax = int(xmin.item()), int(ymin.item()), int(xmax.item()), int(ymax.item())
                box.probability = conf.item()
                box.Class = names[int(cls)]
                boxes.append(box)
                
    return boxes
    
def draw_boxes(img, boxes):
    for box in boxes:
        cv2.rectangle(img, (box.xmin, box.ymin), (box.xmax, box.ymax), (0, 255, 0), 2)
        cv2.putText(img, f'{box.Class} {box.probability:.2f}', (box.xmin, box.ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)
    return img

    
def image_cb(msg, bridge, weights, device, pub, img_pub, confidence):
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    boxes = detect(weights=weights, source=cv_image, device=device, confidence_threshold=confidence)
    bounding_boxes = BoundingBoxes()
    bounding_boxes.header = msg.header
    for box in boxes:
        bounding_boxes.bounding_boxes.append(box)
    pub.publish(bounding_boxes)
    img_pub.publish(bridge.cv2_to_imgmsg(draw_boxes(cv_image, boxes), 'bgr8'))

    
def main():
    set_logging(verbose=False)
    weights = rospy.get_param('~weights_path')
    device = rospy.get_param('~device', '')
    confidence = rospy.get_param('~confidence', 0.85)
    bridge = CvBridge()
    bbxs_pub = rospy.Publisher('zal_yolo_detect_bbx', BoundingBoxes, queue_size=1)
    anotated_pub = rospy.Publisher('zal_yolo_detect_img', Image, queue_size=1)
    rospy.Subscriber('camera/image_raw', Image,
        lambda msg: image_cb(msg, bridge, weights, device, bbxs_pub, anotated_pub, confidence)
    )
    rospy.spin()
    
if __name__ == '__main__':
    main()
