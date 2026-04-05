import cv2
import numpy as np
from rknn.api import RKNN

# 参数定义
RKNN_MODEL = 'best.rknn'
IMG_PATH = './whatermelondata796.jpg'
IMG_SIZE = 640
OBJ_THRESH = 0.25
NMS_THRESH = 0.45
CLASSES = ("cokedata", "milkdata", "potatodata", "pepper", "cakedata", "bananadata", "appledata", "whatermelondata", "tomatodata")


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
    box_confidence = sigmoid(input[..., 4])[..., np.newaxis]
    box_class_probs = sigmoid(input[..., 5:])
    box_xy = sigmoid(input[..., :2]) * 2 - 0.5

    col = np.tile(np.arange(0, grid_w), grid_h).reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
    row = np.tile(np.arange(0, grid_h).reshape(-1, 1), grid_w).reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
    grid = np.concatenate((col, row), axis=-1)

    box_xy += grid
    box_xy *= int(IMG_SIZE / grid_h)
    box_wh = np.square(sigmoid(input[..., 2:4]) * 2)
    box_wh *= anchors
    box = np.concatenate((box_xy, box_wh), axis=-1)
    return box, box_confidence, box_class_probs


def filter_boxes(boxes, box_confidences, box_class_probs):
    boxes = boxes.reshape(-1, 4)
    box_confidences = box_confidences.reshape(-1)
    box_class_probs = box_class_probs.reshape(-1, box_class_probs.shape[-1])

    pos = np.where(box_confidences >= OBJ_THRESH)
    boxes = boxes[pos]
    box_confidences = box_confidences[pos]
    box_class_probs = box_class_probs[pos]

    class_scores = np.max(box_class_probs, axis=-1)
    class_ids = np.argmax(box_class_probs, axis=-1)
    pos = np.where(class_scores >= OBJ_THRESH)

    return boxes[pos], class_ids[pos], (class_scores * box_confidences)[pos]


def nms_boxes(boxes, scores):
    x1, y1 = boxes[:, 0], boxes[:, 1]
    x2, y2 = boxes[:, 2], boxes[:, 3]
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]
    keep = []

    while order.size > 0:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0.0, xx2 - xx1 + 1e-5)
        h = np.maximum(0.0, yy2 - yy1 + 1e-5)
        inter = w * h
        ovr = inter / (areas[i] + areas[order[1:]] - inter)

        inds = np.where(ovr <= NMS_THRESH)[0]
        order = order[inds + 1]

    return np.array(keep)


def yolov5_post_process(outputs):
    masks = [[0, 1, 2], [3, 4, 5], [6, 7, 8]]
    anchors = [[10, 13], [16, 30], [33, 23],
               [30, 61], [62, 45], [59, 119],
               [116, 90], [156, 198], [373, 326]]

    boxes, classes, scores = [], [], []
    for output, mask in zip(outputs, masks):
        b, c, s = process(output, mask, anchors)
        b, c, s = filter_boxes(b, c, s)
        boxes.append(b)
        classes.append(c)
        scores.append(s)

    boxes = np.concatenate(boxes)
    boxes = xywh2xyxy(boxes)
    classes = np.concatenate(classes)
    scores = np.concatenate(scores)

    final_boxes, final_classes, final_scores = [], [], []
    for cls in set(classes):
        idxs = np.where(classes == cls)
        keep = nms_boxes(boxes[idxs], scores[idxs])
        final_boxes.append(boxes[idxs][keep])
        final_classes.append(classes[idxs][keep])
        final_scores.append(scores[idxs][keep])

    if not final_boxes:
        return None, None, None

    return np.concatenate(final_boxes), np.concatenate(final_classes), np.concatenate(final_scores)


def draw(image, boxes, scores, classes):
    for box, score, cl in zip(boxes, scores, classes):
        x1, y1, x2, y2 = map(int, box)
        label = f'{CLASSES[cl]} {score:.2f}'
        cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.putText(image, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        print(f'class: {CLASSES[cl]}, score: {score:.2f}, box: [{x1}, {y1}, {x2}, {y2}]')


def main():
    rknn = RKNN(verbose=False)

    print('--> Load RKNN model')
    if rknn.load_rknn(RKNN_MODEL) != 0:
        print('Failed to load RKNN model.')
        return

    print('--> Init runtime')
    if rknn.init_runtime() != 0:
        print('Failed to init runtime.')
        return

    print('--> Load image')
    img = cv2.imread(IMG_PATH)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (IMG_SIZE, IMG_SIZE))

    print('--> Inference')
    outputs = rknn.inference(inputs=[img])
    print('Inference done.')

    # 保存输出文件（debug）
    for i, out in enumerate(outputs):
        np.save(f'output_{i}.npy', out)

    # 后处理
    inputs = [np.transpose(out.reshape([3, -1] + list(out.shape[-2:])), (2, 3, 0, 1)) for out in outputs]
    boxes, classes, scores = yolov5_post_process(inputs)

    # 可视化并保存图像
    img_draw = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if boxes is not None:
        draw(img_draw, boxes, scores, classes)
        cv2.imwrite('result.jpg', img_draw)
        print('Result image saved to result.jpg')
    else:
        print('No objects detected.')

    rknn.release()


if __name__ == '__main__':
    main()

