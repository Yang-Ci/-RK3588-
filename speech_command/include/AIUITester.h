#ifndef AIUITESTER_H_
#define AIUITESTER_H_

#include <aiui/AIUI.h>
#include <FileUtil.h>
#include <WriteAudioThread.h>
#include <AudioRecorder.h>
#include <TestListener.h>
#include <AudioPlayer.h>
#include <msp_cmn.h>
#include <iostream>
#include <Global.h>
#include "jsoncpp/json/json.h"
#include <RingBuffer.h>
#include "qisr.h"
#include "msp_errors.h"
#include <string>
#include "alsa/asoundlib.h"
#include <stdio.h>
#include <stdlib.h>
#include <numeric>
#include <signal.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>

extern ros::Publisher wakeup_pub;

typedef void (*TEST_CALLBACK)(); 
static TEST_CALLBACK testCallback;

enum class MsgType : unsigned char {
    Shake = 0x01,
    WIFI_SETTING = 0x02,
    AIUI_SETTING = 0x03,
    AIUI_MSG = 0x04,
    CONTROL = 0x05,
    CUSTOM_MGS = 0x2A,
    CONFIRM = 0xff
};

struct MsgPacket
{
    unsigned char uid;
    unsigned char type;
    unsigned short size;
    unsigned short sid;
    std::string bytes;
};

class AIUITester
{
private:
    IAIUIAgent *agent;
    TestListener listener;
    AudioRecorder *audioRecorder;
    AudioPlayer *audioPlayer;

public:
    AIUITester();
    ~AIUITester();
    void destory();
    static TEST_CALLBACK testCallback;

private:
    void showIntroduction(bool detail);
    void createAgent();
    void wakeup();
    void start();
    void stop();
    void recorder_creat();
    void recorder_start();
    void recorder_stop();
    void stopWriteThread();
    void reset();
    void buildGrammar();
    void updateLocalLexicon();

public:
    void readCmd();
    void test();
    void bind(TEST_CALLBACK callback);
};

void gTTS(std::string text);

#endif  // AIUITESTER_H_
