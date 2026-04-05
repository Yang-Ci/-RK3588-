#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from detection_msgs.msg import BoundingBoxes

class YOLOObjectParser:
    def __init__(self):
        rospy.init_node("yolo_object_parser")
        self.sub = rospy.Subscriber("/yolov5/detections", BoundingBoxes, self.callback)
        self.pub = rospy.Publisher("/yolo_objects", String, queue_size=1)
        rospy.loginfo("? yolo_object_parser 节点已启动，监听 YOLO 识别结果...")

    def callback(self, msg):
        object_names = set()
        for box in msg.bounding_boxes:
            object_names.add(box.Class.lower())  # 转小写，避免大小写冲突
        if object_names:
            msg_str = ",".join(sorted(object_names))
            self.pub.publish(msg_str)
            rospy.loginfo("?? 发布识别结果: %s", msg_str)

if __name__ == "__main__":
    YOLOObjectParser()
    rospy.spin()
