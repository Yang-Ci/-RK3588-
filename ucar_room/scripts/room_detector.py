#!/usr/bin/env python3
# coding=utf-8

import rospy
import json
import os
from std_msgs.msg import String, Bool

# 分类信息（与任务类别匹配）
category_to_objects = {
    "fruit": ["apple", "banana", "watermelon"],
    "vegetable": ["tomato", "potato", "pepper"],
    "dessert": ["milk", "cake", "coke"]
}

class RoomDetector:
    def __init__(self):
        rospy.init_node("room_detector")
        self.category = None
        self.detected_object2 = None
        self.detected_room = None

        # 订阅话题配置
        self.sub_category = rospy.Subscriber(
            "/task_status/category",  # 任务类别话题
            String,
            self.category_cb
        )
        self.sub_detection = rospy.Subscriber(
            "/sim_detected",  # 识别结果话题
            String,
            self.detection_cb
        )
        
        # 订阅C++节点发送的音频播放指令
        self.sub_audio_command = rospy.Subscriber(
            "/play_audio_command",
            String,
            self.audio_command_cb
        )

        # 发布任务更新话题
        self.update_pub = rospy.Publisher("/task_update", String, queue_size=10)
        # 新增：发布音频播放完成反馈
        self.audio_done_pub = rospy.Publisher("/audio_played_done", Bool, queue_size=10)

        # 音频文件配置
        self.audio_dir = "/home/iflytek/ucar_ws/src/speech_command/audio/all"
        self.audio_files = {
            "A": "sim_A.wav",
            "B": "sim_B.wav",
            "C": "sim_C.wav"
        }

        rospy.loginfo("【room_detector】初始化完成，等待指令...")
        rospy.loginfo(f"音频文件目录: {self.audio_dir}")
        rospy.spin()

    def category_cb(self, msg):
        self.category = msg.data.strip().lower()
        rospy.loginfo(f"当前任务类别: {self.category}")

    def detection_cb(self, msg):
        if not self.category:
            rospy.logwarn("未接收任务类别，忽略识别结果")
            return

        try:
            data = json.loads(msg.data)
            room = data.get("room", "").upper()
            detected_objects = [x.lower() for x in data.get("objects", [])]

            # 处理房间信息
            if room and room != self.detected_room:
                self.detected_room = room
                self.publish_update("room", room)

            # 处理物体信息
            for obj in detected_objects:
                if obj in category_to_objects.get(self.category, []):
                    if obj != self.detected_object2:
                        self.detected_object2 = obj
                        self.publish_update("object2", obj)
                    break

        except json.JSONDecodeError:
            rospy.logerr("识别结果不是有效的JSON格式")
        except Exception as e:
            rospy.logerr(f"解析识别结果失败: {str(e)}")

    # 处理C++节点发送的音频播放指令
    def audio_command_cb(self, msg):
        command = msg.data.strip().upper()
        if command in ["A", "B", "C"]:
            rospy.loginfo(f"收到音频播放指令: {command}")
            # 播放对应的音频文件
            self.play_audio(command)
        else:
            rospy.logwarn(f"收到无效的音频播放指令: {command}")

    def play_audio(self, audio_key):
        """播放指定音频文件并发送完成反馈"""
        audio_filename = self.audio_files.get(audio_key)
        if not audio_filename:
            rospy.logwarn(f"无对应音频配置: {audio_key}")
            # 即使失败也发送反馈，避免C++一直等待
            self.audio_done_pub.publish(Bool(True))
            return

        audio_path = os.path.join(self.audio_dir, audio_filename)
        if os.path.exists(audio_path):
            rospy.loginfo(f"播放音频: {audio_filename}")
            # 播放音频
            result = os.system(f"aplay '{audio_path}'")
            # 播放完成后发送反馈信号给C++节点
            if result == 0:
                rospy.loginfo("音频播放成功")
                self.audio_done_pub.publish(Bool(True))
            else:
                rospy.logerror("音频播放失败")
                # 即使失败也发送反馈，避免C++一直等待
                self.audio_done_pub.publish(Bool(True))
        else:
            rospy.logwarn(f"音频文件不存在: {audio_path}")
            # 文件不存在也发送反馈
            self.audio_done_pub.publish(Bool(True))

    def publish_update(self, key, value):
        msg = json.dumps({"key": key, "value": value})
        self.update_pub.publish(msg)
        rospy.loginfo(f"发布任务更新: {key} = {value}")

if __name__ == "__main__":
    RoomDetector()
    