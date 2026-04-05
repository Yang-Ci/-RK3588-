#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import os

class TrafficAudio:
    def __init__(self):
        rospy.init_node("traffic_audio")

        self.light1 = None
        self.light2 = None
        self.last_played = None  # 防止重复播放

        rospy.Subscriber("/traffic_light_1", String, self.cb1)
        rospy.Subscriber("/traffic_light_2", String, self.cb2)

        rospy.loginfo("[traffic_audio] 节点已启动，监听红绿灯状态...")
        rospy.spin()

    def cb1(self, msg):
        self.light1 = msg.data.lower()
        self.judge_and_play()

    def cb2(self, msg):
        self.light2 = msg.data.lower()
        self.judge_and_play()

    def judge_and_play(self):
        if self.light1 is None or self.light2 is None:
            return  # 等待两个都到位

        # 判断逻辑
        if self.light1 == "red" or self.light2 == "green":
            target = "road_2.wav"
        else:
            target = "road_1.wav"

        if target != self.last_played:
            self.play_audio(target)
            self.last_played = target

    def play_audio(self, filename):
        path = os.path.expanduser("~/ucar_ws/src/speech_command/audio/all/{}".format(filename))
        if os.path.exists(path):
            rospy.loginfo("[traffic_audio] 播放语音: {}".format(filename))
            os.system("aplay '{}'".format(path))
        else:
            rospy.logwarn("[traffic_audio] 找不到语音文件: {}".format(path))

if __name__ == "__main__":
    TrafficAudio()

