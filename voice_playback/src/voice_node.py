#!/usr/bin/env python3
import rospy
import subprocess
import os
import yaml  # 用于解析YAML配置文件
from std_msgs.msg import String
from rospkg import RosPack

class VoicePlayer:
    def __init__(self):
        # 1. 获取功能包路径和音频目录
        self.package_path = RosPack().get_path('voice_playback')
        self.audio_dir = os.path.join(self.package_path, 'audio')
        rospy.loginfo(f"🎵 音频文件目录: {self.audio_dir}")

        # 2. 读取配置文件路径（从参数获取）
        self.config_path = rospy.get_param('~topics', '')  # 接收外部传入的配置路径
        self.config = self.load_config()  # 加载并解析配置

        # 3. 动态订阅所有话题
        self.subscribers = {}
        for topic, params in self.config.items():
            self.subscribe_topic(topic, params)

        rospy.loginfo(f"✅ 初始化完成，已订阅 {len(self.subscribers)} 个话题")

    def load_config(self):
        """加载并解析YAML配置文件，若失败则用默认配置"""
        # 若未提供配置路径，使用默认配置
        if not self.config_path or not os.path.exists(self.config_path):
            rospy.logwarn(f"⚠️ 配置文件不存在或路径错误: {self.config_path}，使用默认配置")
            return {
                '/task_type': {'prefix': 'task_', 'suffix': '', 'msg_type': 'String'},
                '/pick_complete': {'prefix': 'pick_', 'suffix': '', 'msg_type': 'String'},
                '/sim_complete': {'prefix': 'sim_', 'suffix': '', 'msg_type': 'String'},
                '/road_open': {'prefix': 'road_', 'suffix': '', 'msg_type': 'String'},
                '/final_settlement': {'prefix': 'final_', 'suffix': '', 'msg_type': 'String'}
            }

        # 解析YAML文件
        try:
            with open(self.config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            rospy.loginfo(f"✅ 成功加载配置文件: {self.config_path}")
            return config
        except Exception as e:
            rospy.logerr(f"❌ 解析配置文件失败: {str(e)}，使用默认配置")
            return self.load_config()  # 递归使用默认配置

    def subscribe_topic(self, topic_name, params):
        """订阅话题并绑定回调函数"""
        # 选择消息类型（目前仅支持String）
        msg_class = String if params['msg_type'] == 'String' else String

        # 定义回调函数：收到消息后播放对应音频
        def callback(msg):
            try:
                data = msg.data  # 提取消息内容（如"蔬菜"、"西红柿"）
                # 拼接音频文件名：前缀+消息内容+后缀
                audio_filename = f"{params['prefix']}{data}{params['suffix']}"
                self.play_audio(audio_filename)
            except Exception as e:
                rospy.logerr(f"❌ 处理话题 {topic_name} 失败: {str(e)}")

        # 创建订阅者
        self.subscribers[topic_name] = rospy.Subscriber(
            topic_name, msg_class, callback, queue_size=10
        )
        rospy.loginfo(f"🔗 已订阅话题: {topic_name} → 音频格式: {params['prefix']}[消息内容]{params['suffix']}.wav")

    def play_audio(self, filename):
        """播放指定音频文件"""
        audio_path = os.path.join(self.audio_dir, f"{filename}.wav")
        # 检查文件是否存在
        if not os.path.exists(audio_path):
            rospy.logerr(f"❌ 音频文件不存在: {audio_path}")
            return
        # 调用aplay播放（后台运行，不阻塞节点）
        rospy.loginfo(f"▶️ 播放音频: {filename}.wav")
        subprocess.Popen(["aplay", audio_path])

if __name__ == '__main__':
    try:
        rospy.init_node("voice_player_node", anonymous=False)
        voice_player = VoicePlayer()
        rospy.spin()  # 保持节点运行
    except rospy.ROSInterruptException:
        rospy.loginfo("🔴 节点已停止")
