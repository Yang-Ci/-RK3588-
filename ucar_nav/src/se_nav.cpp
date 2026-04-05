#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>  // 新增：用于发布当前到达的航点
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <atomic>
#include <string>
#include <vector>

ros::Publisher tl_detect_pub;         
ros::Subscriber tl_feedback_sub;      
std::string last_tl_feedback;         
std::atomic<bool> waiting_tl_feedback(false);

enum NavState {
    NAV_IDLE,     
    NAV_WAITING,  
    NAV_SUCCESS,  
    NAV_FAILED    
};

std::atomic<NavState> current_state(NAV_IDLE);
std::atomic<int> current_waypoint(1);
const int TOTAL_WAYPOINTS = 10;
bool speech_done = false;
bool sim_done = false;      
bool audio_played = false;  
std::string last_room_feedback;  

ros::Publisher nav_pub;
ros::Publisher sim_pub;
ros::Publisher audio_command_pub;
ros::ServiceClient clear_costmaps_client;
ros::Publisher vel_pub;
ros::Publisher current_waypoint_pub;  // 新增：发布当前到达的航点

// 语音播报完成回调等略...
void SpeechDoneCallback(const std_msgs::Bool &msg) { speech_done = msg.data; }
void SimDoneCallback(const std_msgs::String &msg) { if (msg.data == "done") sim_done = true; }
void RoomCallback(const std_msgs::String::ConstPtr& msg) { last_room_feedback = msg->data; }
void AudioDoneCallback(const std_msgs::Bool::ConstPtr& msg) { if (msg->data) audio_played = true; }
void TrafficLightFeedbackCallback(const std_msgs::String::ConstPtr& msg) {
    last_tl_feedback = msg->data;
    waiting_tl_feedback = false;
}

void start_traffic_light_detection() {
    std_msgs::Bool start_msg;
    start_msg.data = true;
    tl_detect_pub.publish(start_msg);
    last_tl_feedback.clear();
    waiting_tl_feedback = true;
}

// 小幅度移动函数
void jitter_move_once(const std::string& mode="forward") {
    geometry_msgs::Twist cmd;
    if (mode == "forward")      cmd.linear.x = 0.1;
    else if (mode == "backward")cmd.linear.x = -0.1;
    else if (mode == "left")    cmd.angular.z = 0.15;
    else if (mode == "right")   cmd.angular.z = -0.15;
    vel_pub.publish(cmd);
    ros::Duration(0.4).sleep(); // 小幅度移动
    cmd.linear.x = 0;
    cmd.angular.z = 0;
    vel_pub.publish(cmd);
    ros::Duration(0.2).sleep(); // 停稳
}

int handleSpecialWaypoint(int waypoint_id)
{
    switch(waypoint_id) 
    {
        case 2 : {
            ROS_INFO("已到达起始航点2，开始播报任务...");
            while (ros::ok() && !speech_done) {
                ros::spinOnce();
                ros::Duration(0.1).sleep();
            }
            ROS_INFO("语音播报完成，准备前往下一个航点");
            speech_done = false;
            break;
        }
        case 6: {
            ROS_INFO("已到达航点6，开始执行仿真任务...");

            std_msgs::String sim_msg;
            sim_msg.data = "start";
            sim_pub.publish(sim_msg);
            ROS_INFO("已通过/start_simulation话题发布仿真启动信号！");

            // 等待仿真任务完成
            ROS_INFO("等待仿真完成信号...");
            sim_done = false;
            while (ros::ok() && !sim_done) {
                ros::spinOnce();
                ros::Duration(0.2).sleep();
            }
            ROS_INFO("仿真任务已完成");

            // 检查是否有房间反馈
            if (last_room_feedback.empty()) {
                ROS_WARN("未收到房间反馈，无法播放对应音频");
                break;
            }

            // 发送音频播放指令给Python节点
            audio_played = false;
            std_msgs::String audio_msg;
            audio_msg.data = last_room_feedback;
            audio_command_pub.publish(audio_msg);
            ROS_INFO("已发送音频播放指令: %s", last_room_feedback.c_str());

            // 等待音频播放完成
            ROS_INFO("等待音频播放完成...");
            while (ros::ok() && !audio_played) {
                ros::spinOnce();
                ros::Duration(0.2).sleep();
            }

            ROS_INFO("音频播放完成，继续导航");
            break;
        }
        case 7: {
            ROS_INFO("已到达交通灯检测航点7，开始检测红绿灯...");
            int max_jitter_times = 5;
            int jitter_count = 0;
            std::vector<std::string> jitter_modes = {"forward", "backward", "left", "right"};
            int jitter_mode_idx = 0;
            bool detection_over = false;

            while (ros::ok() && !detection_over) {
                start_traffic_light_detection();
                ros::Time start_wait = ros::Time::now();
                ros::Duration max_wait(8.0);

                waiting_tl_feedback = true;
                while (ros::ok() && waiting_tl_feedback) {
                    ros::spinOnce();
                    if ((ros::Time::now() - start_wait) > max_wait) {
                        ROS_WARN("红绿灯检测超时，默认Red");
                        last_tl_feedback = "Red";
                        waiting_tl_feedback = false;
                        break;
                    }
                    ros::Duration(0.2).sleep();
                }

                if (last_tl_feedback == "Red") {
                    ROS_INFO("红灯，准备前往航点8继续检测...");
                    return -1;  // 默认流程，去8
                } else if (last_tl_feedback == "Green") {
                    ROS_INFO("绿灯，直接前往导航点9进入巡线阶段，不去8/10！");
                    return 9; // 跳转去9
                } else {
                    ROS_WARN("未知检测结果，尝试辅助移动后再检测...");
                    if (jitter_count < max_jitter_times) {
                        jitter_move_once(jitter_modes[jitter_mode_idx % jitter_modes.size()]);
                        jitter_mode_idx++;
                        jitter_count++;
                        continue; // 再检测一次
                    } else {
                        ROS_WARN("多次辅助移动后仍无法检测红绿灯，默认去8");
                        return -1; // 尝试N次后，强行去8
                    }
                }
            }
            // 如果while循环退出，返回默认值
            return -1;
        }
        case 8: {
            ROS_INFO("已到达交通灯检测航点8，开始检测红绿灯...");
            int max_jitter_times = 5;
            int jitter_count = 0;
            std::vector<std::string> jitter_modes = {"forward", "backward", "left", "right"};
            int jitter_mode_idx = 0;
            bool detection_over = false;

            while (ros::ok() && !detection_over) {
                start_traffic_light_detection();
                ros::Time start_wait = ros::Time::now();
                ros::Duration max_wait(8.0);

                waiting_tl_feedback = true;
                while (ros::ok() && waiting_tl_feedback) {
                    ros::spinOnce();
                    if ((ros::Time::now() - start_wait) > max_wait) {
                        ROS_WARN("红绿灯检测超时，默认Red");
                        last_tl_feedback = "Red";
                        waiting_tl_feedback = false;
                        break;
                    }
                    ros::Duration(0.2).sleep();
                }

                if (last_tl_feedback == "Green") {
                    ROS_INFO("8点识别到绿灯，直接前往导航点10进入巡线阶段，不去9！");
                    return 10;
                } else if (last_tl_feedback == "Red") {
                    ROS_INFO("依然红灯，保持等待或可人工介入（可扩展）");
                    return 0; // 停在8，主循环不自增
                } else {
                    ROS_WARN("未知检测结果，尝试辅助移动后再检测...");
                    if (jitter_count < max_jitter_times) {
                        jitter_move_once(jitter_modes[jitter_mode_idx % jitter_modes.size()]);
                        jitter_mode_idx++;
                        jitter_count++;
                        continue; // 再检测一次
                    } else {
                        ROS_WARN("多次辅助移动后仍无法检测红绿灯，保持等待");
                        return 0; // 尝试N次后，停在8
                    }
                }
            }
            // 如果while循环退出，返回默认值
            return 0;
        }
        case 9: {
            ROS_INFO("已到达导航点9，进入巡线阶段，结束导航节点！");
            system("rosnode kill /detector");
            ros::Duration(1.0).sleep();
            system("roslaunch line line_follower.launch &");
            ros::shutdown();
            return -2;
        }
        case 10: {
            ROS_INFO("已到达导航点10，进入巡线阶段，结束导航节点！");
            system("rosnode kill /detector");
            ros::Duration(1.0).sleep();
            system("roslaunch line line_follower.launch &");
            ros::shutdown();
            return -2;
        }
        default: {
            return -1;
        }
    }
    // 添加默认返回值
    return -1;
}

void NavResultCallback(const std_msgs::String &msg) {
    if(msg.data == "done") current_state = NAV_SUCCESS;
    else current_state = NAV_FAILED;
}

bool sendNavigationGoal(int waypoint_id) {
    if(waypoint_id < 1 || waypoint_id > TOTAL_WAYPOINTS) return false;
    std_msgs::String nav_msg;
    nav_msg.data = std::to_string(waypoint_id);
    nav_pub.publish(nav_msg);
    current_state = NAV_WAITING;
    return true;
}

bool clearCostmaps() {
    std_srvs::Empty empty_srv;
    if(clear_costmaps_client.call(empty_srv)) return true;
    else return false;
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "se_nav");
    ros::NodeHandle n;

    nav_pub = n.advertise<std_msgs::String>("/waterplus/navi_waypoint", 10);
    sim_pub = n.advertise<std_msgs::String>("/start_simulation", 10, true);
    audio_command_pub = n.advertise<std_msgs::String>("/play_audio_command", 10, true);
    current_waypoint_pub = n.advertise<std_msgs::Int32>("/current_waypoint", 10);  // 初始化当前航点发布器

    tl_detect_pub = n.advertise<std_msgs::Bool>("/start_traffic_light_detection", 1);
    tl_feedback_sub = n.subscribe("/traffic_light_feedback", 1, TrafficLightFeedbackCallback);

    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::Subscriber res_sub = n.subscribe("/waterplus/navi_result", 10, NavResultCallback);
    ros::Subscriber speech_sub = n.subscribe("/speech_done", 1, SpeechDoneCallback);
    ros::Subscriber sim_sub = n.subscribe("/simulation_done", 1, SimDoneCallback);
    ros::Subscriber room_sub = n.subscribe("/room_feedback", 1, RoomCallback);
    ros::Subscriber audio_done_sub = n.subscribe("/audio_played_done", 1, AudioDoneCallback);

    clear_costmaps_client = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

    ros::Rate rate(1);
    int retry_count = 0;
    const int MAX_RETRIES = 3;

    ros::Duration(1.0).sleep();

    sendNavigationGoal(current_waypoint.load());

    while(ros::ok())
    {
        ros::spinOnce();

        NavState state = current_state.load();
        switch(state)
        {
            case NAV_SUCCESS:
            {
                int wp = current_waypoint.load();
                // 关键修改：仅在到达当前航点后，发布当前航点信息（确保task_executor收到的是"已到达"状态）
                std_msgs::Int32 wp_msg;
                wp_msg.data = wp;
                current_waypoint_pub.publish(wp_msg);
                ROS_INFO("已到达航点 %d，已发布当前航点信息", wp);

                int next_wp = handleSpecialWaypoint(wp);

                retry_count = 0;

                if(wp >= TOTAL_WAYPOINTS) return 0;

                if (next_wp == -2) return 0;
                else if (next_wp > 0) current_waypoint = next_wp;
                else if (next_wp == -1) current_waypoint++;
                else if (next_wp == 0) {
                    // 保持在当前点
                    ROS_INFO("保持在当前航点，重复检测红绿灯...");
                }

                sendNavigationGoal(current_waypoint.load());
                break;
            }
            case NAV_FAILED:
            {
                int wp = current_waypoint.load();
                clearCostmaps();
                retry_count++;
                if(retry_count >= MAX_RETRIES) return -1;
                sendNavigationGoal(wp);
                break;
            }
            case NAV_IDLE:
            case NAV_WAITING:
                break;
        }
        rate.sleep();
    }
    return 0;
}