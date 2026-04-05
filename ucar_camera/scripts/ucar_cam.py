#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Bool, String
import os

last_category = None
speech_done_pub = None
category_pub = None
qrcode_content_pub = None
sub = None  # 用于后续注销订阅

def play_audio(category_en):
    base_path = os.path.expanduser("~/ucar_ws/src/speech_command/audio/all")
    filename = "task_{}.wav".format(category_en.lower())
    filepath = os.path.join(base_path, filename)
    if os.path.exists(filepath):
        os.system("aplay '{}'".format(filepath))
        rospy.sleep(0.5)
        speech_done_pub.publish(True)
    else:
        rospy.logwarn("音频文件不存在: {}".format(filepath))

def callback(data):
    global last_category, category_pub, qrcode_content_pub, sub
    bridge = CvBridge()
    try:
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
    except Exception as e:
        rospy.logwarn(f"图像转换失败: {e}")
        return

    if frame is None or frame.size == 0:
        rospy.logwarn("接收到空图像，跳过处理")
        return

    frame = cv2.flip(frame, 1)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    h, w = gray.shape[:2]
    if h <= 0 or w <= 0:
        rospy.logwarn("灰度图像尺寸无效，跳过处理")
        return

    detector = cv2.QRCodeDetector()
    try:
        data_str, points, _ = detector.detectAndDecode(gray)
    except Exception as e:
        rospy.logerr(f"二维码检测异常: {e}")
        return

    content_to_info = {
        "Vegetable": u"蔬菜",
        "Fruit": u"水果",
        "Dessert": u"甜品"
    }

    if data_str:
        qrcode_content_pub.publish(String(data=data_str))
        rospy.loginfo(f"已发布二维码原始内容: {data_str}")

        if data_str in content_to_info and data_str != last_category:
            last_category = data_str
            category_pub.publish(String(data=data_str))
            rospy.set_param("/task_status/category", data_str)

            rospy.loginfo(f"[扫码识别] 识别到的物体：{content_to_info[data_str]}")
            rospy.loginfo(f"已设置任务类别为: {data_str}")
            play_audio(data_str)

            # ✅ 识别完成后，取消订阅并退出节点
            sub.unregister()
            cv2.destroyAllWindows()
            rospy.loginfo("识别完成，二维码节点退出，摄像头继续供其他节点使用")
            rospy.signal_shutdown("任务识别完成")

    cv2.imshow("QR Code Scanner", frame)
    cv2.waitKey(1)

    #rospy.loginfo("图像已接受并处理完成")

def qrcode_demo():
    global speech_done_pub, category_pub, qrcode_content_pub, sub
    rospy.init_node('qrcode_demo', anonymous=True)
    speech_done_pub = rospy.Publisher('/speech_done', Bool, queue_size=1)
    category_pub = rospy.Publisher('/task_status/category', String, queue_size=1, latch=True)
    qrcode_content_pub = rospy.Publisher('/qrcode_content', String, queue_size=1, latch=True)

    # ✅ 订阅摄像头统一话题
    sub = rospy.Subscriber('/usb_cam/image_raw', Image, callback)
    rospy.loginfo("二维码识别节点已启动，订阅摄像头图像流...")
    rospy.spin()

if __name__ == '__main__':
    qrcode_demo()
