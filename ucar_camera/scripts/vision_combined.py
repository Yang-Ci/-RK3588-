#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import rospy
import numpy as np
import logging
import datetime as dt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Bool
from rknnlite.api import RKNNLite
import os
import rosgraph.roslogging

# --------------------------
# 1. 日志配置（来自video.py，必须优先定义）
# --------------------------
def custom_configure_logging(name, level=None, filename=None):
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.INFO)
    if not any(isinstance(h, logging.StreamHandler) for h in root_logger.handlers):
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        console_handler.setFormatter(formatter)
        root_logger.addHandler(console_handler)
    return ""
rosgraph.roslogging.configure_logging = custom_configure_logging

# --------------------------
# 2. 全局变量（区分命名空间，避免重名）
# --------------------------
# 二维码识别相关
qr_last_category = None  # 用qr_前缀区分
qr_content_map = {
    "Vegetable": u"蔬菜",
    "Fruit": u"水果",
    "Dessert": u"甜品"
}

# YOLO识别相关
yolo_rknn = None  # 用yolo_前缀区分
yolo_model = 'best.rknn'
yolo_img_size = 640
yolo_obj_thresh = 0.25
yolo_nms_thresh = 0.45
yolo_classes = (
    "cokedata", "milkdata", "potatodata", "pepper", "cakedata",
    "bananadata", "appledata", "whatermelondata", "tomatodata"
)

# 共享资源
bridge = CvBridge()  # 共用一个CV桥接器
# ROS发布者（明确区分话题）
qr_category_pub = None       # 二维码结果：/task_status/category
qr_speech_done_pub = None    # 二维码语音完成：/speech_done
yolo_objects_pub = None      # YOLO结果：/yolo_objects

# --------------------------
# 3. 二维码识别逻辑（来自ucar_cam.py）
# --------------------------
def qr_play_audio(category):
    """播放二维码任务提示音"""
    base_path = os.path.expanduser("~/ucar_ws/src/speech_command/audio/all")
    filepath = os.path.join(base_path, f"task_{category.lower()}.wav")
    if os.path.exists(filepath):
        os.system(f"aplay '{filepath}'")
        rospy.sleep(0.5)
        qr_speech_done_pub.publish(True)
    else:
        rospy.logwarn(f"二维码音频缺失: {filepath}")

def qr_detect(frame):
    """识别二维码并返回类别"""
    global qr_last_category
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detector = cv2.QRCodeDetector()
    try:
        data_str, _, _ = detector.detectAndDecode(gray)
    except Exception as e:
        rospy.logerr(f"二维码识别失败: {e}")
        return None
    if data_str in qr_content_map and data_str != qr_last_category:
        qr_last_category = data_str
        return data_str
    return None

# --------------------------
# 4. YOLO识别逻辑（来自video.py）
# --------------------------
def yolo_sigmoid(x):
    return 1 / (1 + np.exp(-x))

def yolo_xywh2xyxy(x):
    y = np.copy(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2
    y[:, 1] = x[:, 1] - x[:, 3] / 2
    y[:, 2] = x[:, 0] + x[:, 2] / 2
    y[:, 3] = x[:, 1] + x[:, 3] / 2
    return y

def yolo_process(input, mask, anchors):
    anchors = [anchors[i] for i in mask]
    grid_h, grid_w = map(int, input.shape[0:2])
    box_confidence = yolo_sigmoid(input[..., 4])
    box_confidence = np.expand_dims(box_confidence, axis=-1)
    box_class_probs = yolo_sigmoid(input[..., 5:])
    box_xy = yolo_sigmoid(input[..., :2]) * 2 - 0.5

    col = np.tile(np.arange(0, grid_w), grid_w).reshape(-1, grid_w)
    row = np.tile(np.arange(0, grid_h).reshape(-1, 1), grid_h)
    col = col.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
    row = row.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
    grid = np.concatenate((col, row), axis=-1)
    box_xy += grid
    box_xy *= int(yolo_img_size / grid_h)
    box_wh = np.power(yolo_sigmoid(input[..., 2:4]) * 2, 2)
    box_wh = box_wh * anchors
    box = np.concatenate((box_xy, box_wh), axis=-1)
    return box, box_confidence, box_class_probs

def yolo_filter_boxes(boxes, box_confidences, box_class_probs):
    boxes = boxes.reshape(-1, 4)
    box_confidences = box_confidences.reshape(-1)
    box_class_probs = box_class_probs.reshape(-1, box_class_probs.shape[-1])
    _box_pos = np.where(box_confidences >= yolo_obj_thresh)
    boxes = boxes[_box_pos]
    box_confidences = box_confidences[_box_pos]
    box_class_probs = box_class_probs[_box_pos]
    class_max_score = np.max(box_class_probs, axis=-1)
    classes = np.argmax(box_class_probs, axis=-1)
    _class_pos = np.where(class_max_score >= yolo_obj_thresh)
    boxes = boxes[_class_pos]
    classes = classes[_class_pos]
    scores = (class_max_score * box_confidences)[_class_pos]
    return boxes, classes, scores

def yolo_nms_boxes(boxes, scores):
    x = boxes[:, 0]
    y = boxes[:, 1]
    w = boxes[:, 2] - boxes[:, 0]
    h = boxes[:, 3] - boxes[:, 1]
    areas = w * h
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x[i], x[order[1:]])
        yy1 = np.maximum(y[i], y[order[1:]])
        xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
        yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])
        w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
        h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
        inter = w1 * h1
        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= yolo_nms_thresh)[0]
        order = order[inds + 1]
    keep = np.array(keep)
    return keep

def yolo_post_process(input_data):
    masks = [[0, 1, 2], [3, 4, 5], [6, 7, 8]]
    anchors = [
        [10, 13], [16, 30], [33, 23],
        [30, 61], [62, 45], [59, 119],
        [116, 90], [156, 198], [373, 326]
    ]
    boxes, classes, scores = [], [], []
    for input, mask in zip(input_data, masks):
        b, c, s = yolo_process(input, mask, anchors)
        b, c, s = yolo_filter_boxes(b, c, s)
        boxes.append(b)
        classes.append(c)
        scores.append(s)
    if len(boxes) == 0:
        return None, None, None
    boxes = np.concatenate(boxes)
    boxes = yolo_xywh2xyxy(boxes)
    classes = np.concatenate(classes)
    scores = np.concatenate(scores)
    nboxes, nclasses, nscores = [], [], []
    for c in set(classes):
        inds = np.where(classes == c)
        b = boxes[inds]
        c = classes[inds]
        s = scores[inds]
        keep = yolo_nms_boxes(b, s)
        nboxes.append(b[keep])
        nclasses.append(c[keep])
        nscores.append(s[keep])
    if not nclasses and not nscores:
        return None, None, None
    return np.concatenate(nboxes), np.concatenate(nclasses), np.concatenate(nscores)

def yolo_draw(image, boxes, scores, classes, fps):
    for box, score, cl in zip(boxes, scores, classes):
        top, left, right, bottom = box
        cv2.rectangle(image, (int(top), int(left)), (int(right), int(bottom)), (255, 0, 0), 2)
        cv2.putText(image, f'{yolo_classes[cl]} {score:.2f}',
                    (int(top), int(left)-6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
    cv2.putText(image, f'fps: {fps}', (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,125,125), 2)

# --------------------------
# 5. 核心：共享图像回调函数
# --------------------------
def image_callback(msg):
    # 1. 转换图像（共享同一帧）
    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logwarn(f"图像转换失败: {e}")
        return

    # 2. 二维码识别（优先处理）
    qr_result = qr_detect(frame)
    if qr_result:
        rospy.loginfo(f"识别到二维码类别: {qr_content_map[qr_result]}")
        qr_category_pub.publish(String(data=qr_result))
        qr_play_audio(qr_result)

    # 3. YOLO识别（并行处理同一帧）
    start_time = dt.datetime.utcnow()
    img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img_resized = cv2.resize(img_rgb, (yolo_img_size, yolo_img_size))
    
    # 模型推理
    outputs = yolo_rknn.inference(inputs=[img_resized])
    fps = round(1000000 / ( (dt.datetime.utcnow() - start_time).microseconds + 1 ))
    
    # 后处理
    input0 = outputs[0].reshape([3, -1] + list(outputs[0].shape[-2:]))
    input1 = outputs[1].reshape([3, -1] + list(outputs[1].shape[-2:]))
    input2 = outputs[2].reshape([3, -1] + list(outputs[2].shape[-2:]))
    input_data = [
        np.transpose(input0, (2, 3, 0, 1)),
        np.transpose(input1, (2, 3, 0, 1)),
        np.transpose(input2, (2, 3, 0, 1))
    ]
    boxes, classes, scores = yolo_post_process(input_data)

    # 发布YOLO结果
    if boxes is not None:
        detected = [yolo_classes[cl] for cl in classes]
        yolo_objects_pub.publish(String(data=",".join(set(detected))))
        rospy.loginfo(f"YOLO识别结果: {detected}")
        # 绘制检测框
        img_draw = cv2.cvtColor(img_resized, cv2.COLOR_RGB2BGR)
        yolo_draw(img_draw, boxes, scores, classes, fps)
        cv2.imshow("Combined Vision", img_draw)
    else:
        cv2.putText(frame, "No objects detected", (20, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
        cv2.imshow("Combined Vision", frame)
    cv2.waitKey(1)

# --------------------------
# 6. 初始化与主函数
# --------------------------
def main():
    global qr_category_pub, qr_speech_done_pub, yolo_objects_pub, yolo_rknn

    # 初始化ROS节点（唯一节点）
    rospy.init_node('combined_vision', anonymous=True)

    # 初始化发布者（明确区分）
    qr_category_pub = rospy.Publisher('/task_status/category', String, queue_size=1, latch=True)
    qr_speech_done_pub = rospy.Publisher('/speech_done', Bool, queue_size=1)
    yolo_objects_pub = rospy.Publisher('/yolo_objects', String, queue_size=10)

    # 加载YOLO模型（优先初始化）
    yolo_rknn = RKNNLite()
    rospy.loginfo(f"加载RKNN模型: {yolo_model}")
    if yolo_rknn.load_rknn(yolo_model) != 0:
        rospy.logerr("模型加载失败")
        return
    if yolo_rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2) != 0:
        rospy.logerr("NPU初始化失败")
        return
    rospy.loginfo("YOLO模型初始化完成")

    # 订阅摄像头图像（共享源）
    rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
    rospy.loginfo("合并节点启动成功，等待图像输入...")

    # 保持运行
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("用户终止程序")
    finally:
        # 释放资源
        yolo_rknn.release()
        cv2.destroyAllWindows()
        rospy.loginfo("资源已释放")

if __name__ == "__main__":
    main()
