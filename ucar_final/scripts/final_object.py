from std_msgs.msg import Bool

class FinalAnnouncer:
    def __init__(self):
        rospy.init_node("final_object_node")
        self.category = None
        self.obj1 = None
        self.obj2 = None
        self.has_announced = False

        rospy.Subscriber("/task_status/category", String, self.cb_category)
        rospy.Subscriber("/task_status/object1", String, self.cb_obj1)
        rospy.Subscriber("/task_status/object2", String, self.cb_obj2)

        self.speech_done_pub = rospy.Publisher("/speech_done", Bool, queue_size=1)

        rospy.loginfo("final_object_node 已启动，等待三个任务状态...")
        rospy.spin()

    def cb_category(self, msg):
        self.category = msg.data.lower()
        self.try_announce()

    def cb_obj1(self, msg):
        self.obj1 = msg.data.lower()
        self.try_announce()

    def cb_obj2(self, msg):
        self.obj2 = msg.data.lower()
        self.try_announce()

    def try_announce(self):
        if self.has_announced:
            return

        if self.category and self.obj1 and self.obj2:
            objs = sorted([self.obj1, self.obj2])
            filename = f"final_{self.category}_{objs[0]}_{objs[1]}.wav"
            audio_path = os.path.expanduser(f"~/ucar_ws/src/speech_command/audio/all/{filename}")
            if os.path.exists(audio_path):
                rospy.loginfo(f"播放最终音频: {filename}")
                os.system(f"aplay '{audio_path}'")
                self.has_announced = True
                rospy.sleep(0.5)
                self.speech_done_pub.publish(True)
            else:
                rospy.logwarn(f"音频文件不存在: {filename}")
