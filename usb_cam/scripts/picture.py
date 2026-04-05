#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2  
import time  
import os  
  
# 确保图像文件夹存在  
image_folder = "images"  
if not os.path.exists(image_folder):  
    os.makedirs(image_folder)  
  
# 打开摄像头  
cap = cv2.VideoCapture(0)  
  
# 检查摄像头是否成功打开  
if not cap.isOpened():  
    print("无法打开摄像头，请检查设备连接和权限。")  
    exit()  
  
# 初始化计时器  
last_save_time = time.time()  
save_interval = 2  # 保存间隔（秒）  
a = 0  # 帧计数器  
  
try:  
    while True:  
        # 读取一帧图像  
        ret, frame = cap.read()  
        if not ret:  
            print("无法获取摄像头帧。退出...")  
            break  
  
        # 翻转图像，如果需要的话  
        # frame = cv2.flip(frame, 1)  # 1表示水平翻转，如果不需要可以注释掉  
  
        # 显示图像  
        cv2.imshow("Capture", frame)  
  
        # 检查是否到了保存图像的时间  
        if time.time() - last_save_time >= save_interval:  
            a += 1  
            # 保存当前帧为图像到图像文件夹  
            filename = os.path.join(image_folder, "test_{}.png".format(int(time.time())))  
            cv2.imwrite(filename, frame)  
            print("图像已保存为: ",filename,a)   
            last_save_time = time.time()  # 更新最后保存时间  
  
        # 等待按键，如果按下'q'则退出循环  
        if cv2.waitKey(1) & 0xFF == ord('q'):  
            break  
  
finally:  
    # 释放摄像头资源并关闭所有OpenCV窗口  
    cap.release()  
    cv2.destroyAllWindows()
