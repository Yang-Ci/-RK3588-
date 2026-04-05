#!/usr/bin/env python3

import numpy as np
import cv2
import logging
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rknnlite.api import RKNNLite
import datetime as dt
import rosgraph.roslogging

# 替换ROS默认的日志配置函数，避免加载错误配置
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

RKNN_MODEL = rospy.get_param("~rknn_model", "best.rknn")
print("实际加载的RKNN模型路径:", RKNN_MODEL)

IMG_SIZE = 640
OBJ_THRESH = 0.25
NMS_THRESH = 0.45
CLASSES = (
    "cokedata", "milkdata", "potatodata", "pepper", "cakedata",
    "bananadata", "appledata", "whatermelondata", "tomatodata"
)

def sigmoid(x):
    return 1 / (1 + np.exp(-x))

def xywh2xyxy(x):
    y = np.copy(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2
    y[:, 1] = x[:, 1] - x[:, 3] / 2
    y[:, 2] = x[:, 0] + x[:, 2] / 2
    y[:, 3] = x[:, 1] + x[:, 3] / 2
    return y

def process(input, mask, anchors):
    anchors = [anchors[i] for i in mask]
    grid_h, grid_w = map(int, input.shape[0:2])
    box_confidence = sigmoid(input[..., 4])
    box_confidence = np.expand_dims(box_confidence, axis=-1)
    box_class_probs = sigmoid(input[..., 5:])
    box_xy = sigmoid(input[..., :2]) * 2 - 0.5

    col = np.tile(np.arange(0, grid_w), grid_w).reshape(-1, grid_w)
    row = np.tile(np.arange(0, grid_h).reshape(-1, 1), grid_h)
    col = col.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
    row = row.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
    grid = np.concatenate((col, row), axis=-1)
    box_xy += grid
    box_xy *= int(IMG_SIZE / grid_h)
    box_wh = np.power(sigmoid(input[..., 2:4]) * 2, 2)
    box_wh = box_wh * anchors
    box = np.concatenate((box_xy, box_wh), axis=-1)
    return box, box_confidence, box_class_probs

def filter_boxes(boxes, box_confidences, box_class_probs):
    boxes = boxes.reshape(-1, 4)
    box_confidences = box_confidences.reshape(-1)
    box_class_probs = box_class_probs.reshape(-1, box_class_probs.shape[-1])
    _box_pos = np.where(box_confidences >= OBJ_THRESH)
    boxes = boxes[_box_pos]
    box_confidences = box_confidences[_box_pos]
    box_class_probs = box_class_probs[_box_pos]
    class_max_score = np.max(box_class_probs, axis=-1)
    classes = np.argmax(box_class_probs, axis=-1)
    _class_pos = np.where(class_max_score >= OBJ_THRESH)
    boxes = boxes[_class_pos]
    classes = classes[_class_pos]
    scores = (class_max_score * box_confidences)[_class_pos]
    return boxes, classes, scores

def nms_boxes(boxes, scores):
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
        inds = np.where(ovr <= NMS_THRESH)[0]
        order = order[inds + 1]
    keep = np.array(keep)
    return keep

def yolov5_post_process(input_data):
    masks = [[0, 1, 2], [3, 4, 5], [6, 7, 8]]
    anchors = [
        [10, 13], [16, 30], [33, 23],
        [30, 61], [62, 45], [59, 119],
        [116, 90], [156, 198], [373, 326]
    ]
    boxes, classes, scores = [], [], []
    for input, mask in zip(input_data, masks):
        b, c, s = process(input, mask, anchors)
        b, c, s = filter_boxes(b, c, s)
        boxes.append(b)
        classes.append(c)
        scores.append(s)
    if len(boxes) == 0:
        return None, None, None
    boxes = np.concatenate(boxes)
    boxes = xywh2xyxy(boxes)
    classes = np.concatenate(classes)
    scores = np.concatenate(scores)
    nboxes, nclasses, nscores = [], [], []
    for c in set(classes):
        inds = np.where(classes == c)
        b = boxes[inds]
        c = classes[inds]
        s = scores[inds]
        keep = nms_boxes(b, s)
        nboxes.append(b[keep])
        nclasses.append(c[keep])
        nscores.append(s[keep])
    if not nclasses and not nscores:
        return None, None, None
    boxes = np.concatenate(nboxes)
    classes = np.concatenate(nclasses)
    scores = np.concatenate(nscores)
    return boxes, classes, scores

def draw(image, boxes, scores, classes, fps):
    for box, score, cl in zip(boxes, scores, classes):
        top, left, right, bottom = box
        top = int(top)
        left = int(left)
        right = int(right)
        bottom = int(bottom)
        cv2.rectangle(image, (top, left), (right, bottom), (255, 0, 0), 2)
        cv2.putText(image, '{0} {1:.2f}'.format(CLASSES[cl], score),
                    (top, left - 6),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 255), 2)
    cv2.putText(image, f'fps: {fps}', (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 125, 125), 2)

# 全局变量初始化
bridge = CvBridge()
rknn = None
pub = None
last_time = None

def image_callback(msg):
    global rknn, pub, last_time

    try:
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except Exception as e:
        logging.error(f"图像转换失败: {e}")
        return

    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img_resized = cv2.resize(img_rgb, (IMG_SIZE, IMG_SIZE))

    start_time = dt.datetime.utcnow()
    outputs = rknn.inference(inputs=[img_resized])
    duration = dt.datetime.utcnow() - start_time
    fps = round(1000000 / (duration.microseconds + 1))

    input0_data = outputs[0].reshape([3, -1] + list(outputs[0].shape[-2:]))
    input1_data = outputs[1].reshape([3, -1] + list(outputs[1].shape[-2:]))
    input2_data = outputs[2].reshape([3, -1] + list(outputs[2].shape[-2:]))

    input_data = [
        np.transpose(input0_data, (2, 3, 0, 1)),
        np.transpose(input1_data, (2, 3, 0, 1)),
        np.transpose(input2_data, (2, 3, 0, 1))
    ]

    boxes, classes, scores = yolov5_post_process(input_data)

    img_draw = cv2.cvtColor(img_resized, cv2.COLOR_RGB2BGR)

    detected_objects = []
    if boxes is not None:
        for cl in classes:
            detected_objects.append(CLASSES[cl])
        unique_objects = list(set(detected_objects))
        obj_str = ",".join(unique_objects)
        pub.publish(obj_str)
        logging.info(f"发布识别结果: {obj_str}")

        draw(img_draw, boxes, scores, classes, fps)
    else:
        cv2.putText(img_draw, 'No object detected.', (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(img_draw, f'fps: {fps}', (20, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 125, 125), 2)

    cv2.imshow("Real-time Detection", img_draw)
    key = cv2.waitKey(1)
    if key & 0xFF == ord('q') or key == 27:
        rospy.signal_shutdown("用户退出程序")

def main():
    global rknn, pub
    rospy.init_node('yolo_detector', anonymous=True)
    pub = rospy.Publisher('/yolo_objects', String, queue_size=10)

    rknn = RKNNLite()
    logging.info(f"加载RKNN模型: {RKNN_MODEL}")
    ret = rknn.load_rknn(RKNN_MODEL)
    if ret != 0:
        logging.error(f"模型加载失败，错误码: {ret}")
        exit(ret)

    logging.info("初始化NPU运行环境")
    ret = rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)
    if ret != 0:
        logging.error(f"运行环境初始化失败，错误码: {ret}")
        exit(ret)
    logging.info("NPU初始化完成")

    rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
    rospy.loginfo("yolo_detector节点启动，等待图像数据...")
    rospy.spin()

    # 程序退出时释放资源
    cv2.destroyAllWindows()
    rknn.release()
    logging.info("程序退出，资源已释放")

if __name__ == '__main__':
    main()
