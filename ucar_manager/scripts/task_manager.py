#!/usr/bin/env python3
import rospy
import os
from std_msgs.msg import String

AUDIO_DIR = "/home/iflytek/ucar_ws/src/speech_command/audio/all"

category_to_objects = {
    "fruit": ["apple", "banana", "watermelon"],
    "vegetable": ["tomato", "potato", "pepper"],
    "dessert": ["milk", "cake", "coke"]
}

class TaskStatus:
    def __init__(self):
        self.category = None
        self.object1 = None
        self.object2 = None
        self.room = None

    def is_ready_for_final(self):
        return self.category and self.object1 and self.object2

    def get_final_audio_file(self):
        objs = sorted([self.object1, self.object2])
        valid_objs = category_to_objects.get(self.category, [])
        if objs[0] in valid_objs and objs[1] in valid_objs:
            filename = f"final_{self.category}_{objs[0]}_{objs[1]}.wav"
            return os.path.join(AUDIO_DIR, filename)
        else:
            rospy.logwarn("物品不属于类别，无法匹配最终语音文件")
            return None

def play_audio(path):
    if path and os.path.exists(path):
        rospy.loginfo(f"播放音频: {path}")
        os.system(f"aplay '{path}'")
    else:
        rospy.logwarn(f"音频文件不存在: {path}")

def category_cb(msg):
    task_status.category = msg.data.lower()
    rospy.set_param("/task_status/category", task_status.category)
    rospy.loginfo(f"任务类别更新: {task_status.category}")

def object1_cb(msg):
    task_status.object1 = msg.data.lower()
    rospy.set_param("/task_status/object1", task_status.object1)
    rospy.loginfo(f"物品1更新: {task_status.object1}")

def object2_cb(msg):
    task_status.object2 = msg.data.lower()
    rospy.set_param("/task_status/object2", task_status.object2)
    rospy.loginfo(f"物品2更新: {task_status.object2}")

def room_cb(msg):
    task_status.room = msg.data.upper()
    rospy.set_param("/task_status/room", task_status.room)
    rospy.loginfo(f"房间更新: {task_status.room}")

    # 播放房间音频
    room_audio = os.path.join(AUDIO_DIR, f"sim_{task_status.room}.wav")
    play_audio(room_audio)

    # 如果准备好，播放最终音频
    if task_status.is_ready_for_final():
        final_audio = task_status.get_final_audio_file()
        play_audio(final_audio)

if __name__ == '__main__':
    rospy.init_node("task_manager")
    task_status = TaskStatus()

    rospy.Subscriber("/task_status/category", String, category_cb)
    rospy.Subscriber("/task_status/object1", String, object1_cb)
    rospy.Subscriber("/task_status/object2", String, object2_cb)
    rospy.Subscriber("/task_status/room", String, room_cb)

    rospy.loginfo("Task Manager 启动，等待任务状态更新...")
    rospy.spin()
