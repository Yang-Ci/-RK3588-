#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import os
import numpy as np
import rospy
import threading
from std_msgs.msg import Header
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from pyzbar import pyzbar


class UcarCameraQR:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node("ucar_camera_qr", anonymous=True)
        self.bridge = CvBridge()
        
        # 读取参数
        self.img_width = int(rospy.get_param('~image_width', default=640))  # 降低分辨率减轻压力
        self.img_height = int(rospy.get_param('~image_height', default=480))
        self.camera_topic_name = rospy.get_param('~cam_topic_name', default="/ucar_camera/image_raw")
        self.device_path = rospy.get_param('device_path', default="/dev/video0")
        self.cam_pub_rate = int(rospy.get_param('~rate', default=10))  # 降低帧率
        
        # 初始化发布器
        self.cam_pub = rospy.Publisher(self.camera_topic_name, Image, queue_size=2)
        self.qr_result_pub = rospy.Publisher("/qr_code/result", String, queue_size=1)  # 发布QR识别结果
        
        # 初始化Image消息结构
        self.image_msg = Image()
        self.image_msg.header.frame_id = 'camera_link'
        self.image_msg.height = self.img_height
        self.image_msg.width = self.img_width
        self.image_msg.encoding = 'rgb8'
        self.image_msg.is_bigendian = False
        self.image_msg.step = self.img_width * 3
        
        # 摄像头相关
        self.cap = None
        self.running = False
        self.frame_lock = threading.Lock()  # 线程锁保护共享帧
        self.latest_frame = None
        
        # 初始化摄像头
        self.init_camera()
        
        # 启动QR识别线程
        self.qr_thread = threading.Thread(target=self.qr_detection_loop, daemon=True)
        self.qr_thread.start()
        
        # 注册关闭回调
        rospy.on_shutdown(self.shutdown)
        
        rospy.loginfo("UCAR相机节点初始化完成，包含二维码识别功能")

    def init_camera(self):
        """初始化摄像头并检查状态"""
        try:
            self.cap = cv2.VideoCapture(self.device_path)
            if not self.cap.isOpened():
                rospy.logerr(f"无法打开摄像头设备: {self.device_path}")
                rospy.signal_shutdown("摄像头初始化失败")
                return
            
            # 设置摄像头参数
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_height)
            
            # 验证实际分辨率
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            
            if actual_width != self.img_width or actual_height != self.img_height:
                rospy.logwarn(f"摄像头不支持设定的分辨率，已自动调整为: {actual_width}x{actual_height}")
                self.img_width = actual_width
                self.img_height = actual_height
                self.image_msg.width = self.img_width
                self.image_msg.height = self.img_height
                self.image_msg.step = self.img_width * 3
            
            self.running = True
            rospy.loginfo(f"成功打开摄像头: {self.device_path}")
            
        except Exception as e:
            rospy.logerr(f"摄像头初始化错误: {str(e)}")
            rospy.signal_shutdown("摄像头初始化失败")

    def run(self):
        """主循环：读取并发布图像"""
        rate = rospy.Rate(self.cam_pub_rate)
        
        while not rospy.is_shutdown() and self.running and self.cap.isOpened():
            try:
                # 读取帧
                ret, frame = self.cap.read()
                if not ret:
                    rospy.logwarn("无法获取图像帧，尝试重新读取...")
                    rate.sleep()
                    continue
                
                # 处理图像
                frame = cv2.flip(frame, 1)  # 水平翻转
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # 转换为RGB
                
                # 更新共享帧（加锁）
                with self.frame_lock:
                    self.latest_frame = frame_rgb.copy()
                
                # 填充并发布ROS图像消息
                self.image_msg.header.stamp = rospy.Time.now()
                self.image_msg.data = frame_rgb.tobytes()
                self.cam_pub.publish(self.image_msg)
                
                rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"主循环错误: {str(e)}")
                rate.sleep()

    def qr_detection_loop(self):
        """二维码识别线程"""
        rospy.loginfo("二维码识别线程已启动")
        rate = rospy.Rate(5)  # 识别频率低于图像发布频率，减轻CPU负担
        
        while not rospy.is_shutdown() and self.running:
            try:
                # 安全获取最新帧
                frame = None
                with self.frame_lock:
                    if self.latest_frame is not None:
                        frame = self.latest_frame.copy()  # 复制一份避免冲突
                
                if frame is not None:
                    # 转换为灰度图提高识别效率
                    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                    
                    # 检测二维码
                    qr_codes = pyzbar.decode(gray)
                    
                    # 处理识别结果
                    for qr in qr_codes:
                        qr_data = qr.data.decode('utf-8')
                        qr_type = qr.type
                        rospy.loginfo(f"识别到二维码: {qr_data} (类型: {qr_type})")
                        
                        # 发布识别结果
                        self.qr_result_pub.publish(qr_data)
                        
                        # 在图像上绘制二维码边界框（调试用）
                        (x, y, w, h) = qr.rect
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        text = f"{qr_data} ({qr_type})"
                        cv2.putText(frame, text, (x, y - 10), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"二维码识别错误: {str(e)}")
                rate.sleep()

    def shutdown(self):
        """节点关闭时释放资源"""
        rospy.loginfo("正在关闭节点，释放资源...")
        self.running = False
        
        # 等待QR线程结束
        if hasattr(self, 'qr_thread') and self.qr_thread.is_alive():
            self.qr_thread.join(timeout=1.0)
        
        # 释放摄像头
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        
        cv2.destroyAllWindows()
        rospy.loginfo("节点已关闭")


if __name__ == '__main__':
    try:
        # 确保pyzbar库已安装
        try:
            import pyzbar
        except ImportError:
            rospy.logerr("未找到pyzbar库，请先安装: pip install pyzbar")
            exit(1)
            
        # 启动节点
        ucar_cam = UcarCameraQR()
        ucar_cam.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被中断")
    except Exception as e:
        rospy.logerr(f"程序异常退出: {str(e)}")
