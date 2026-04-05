#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <thread>
#include <chrono>
#include <deque>
#include <unordered_map>
#include <cmath>
#include <system_error>

// 状态机定义
enum class State {
    INIT,               // 初始化
    WAIT_FOR_WAYPOINT,  // 等待到达4/5号点
    SCAN_FOR_OBJECT,    // 扫描物品
    SCAN_FOR_BOARD,     // 扫描板子
    NAV_TO_BOARD,       // 导航到板子
    ADJUST_IN_REGION,   // 区域微调
    PLAY_AUDIO,         // 语音播报
    COMPLETE,           // 完成
    FAILED              // 失败
};

// 任务类型映射
const std::unordered_map<std::string, std::pair<std::vector<std::string>, std::string>> TASK_INFO = {
    {"Fruit", {{"apple", "banana", "watermelon"}, "fruit_board"}},
    {"Vegetable", {{"pepper", "tomato", "potato"}, "vegetable_board"}},
    {"Dessert", {{"milk", "cake", "coke"}, "dessert_board"}}
};

// 配置参数
const double REGION_SIZE = 0.6;       // 区域边长（米）
const double POSITION_TOLERANCE = 0.1; // 位置误差容忍度
const double ANGLE_TOLERANCE = 0.1;   // 角度误差容忍度
const int OBJECT_CONFIRM_THRESH = 3;   // 物品识别阈值
const int BOARD_CONFIRM_THRESH = 2;    // 板子识别阈值
const double SCAN_TIMEOUT = 40.0;      // 扫描超时（确保360度旋转）
const double ROTATE_SPEED = 0.2;       // 旋转速度（弧度/秒）
const double FULL_ROTATION = 2 * M_PI; // 360度（弧度）

class TaskExecutor {
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // 订阅者
    ros::Subscriber qr_task_sub_;       // 任务类型
    ros::Subscriber yolo_objects_sub_;  // YOLO识别结果
    ros::Subscriber board_qr_sub_;      // 板子二维码
    ros::Subscriber current_waypoint_sub_;  // 到达航点
    int current_se_nav_waypoint_;
    
    // 发布者
    ros::Publisher cmd_vel_pub_;        // 速度控制
    ros::Publisher yolo_active_pub_;    // YOLO激活控制
    ros::Publisher skip_waypoint_pub_;  // 发布跳过航点信号（用于跳过5号点）
    
    // 状态变量
    State current_state_;
    std::string current_task_;          // 当前任务
    std::string current_waypoint_;      // 当前航点
    std::string target_board_qr_;       // 目标板子二维码
    std::vector<std::string> target_objects_;  // 目标物品列表
    ros::Time scan_start_time_;         // 扫描开始时间
    
    // 识别缓存与标志
    std::deque<std::string> recent_objects_;  // 近期物品
    std::deque<std::string> recent_boards_;   // 近期板子
    bool object_confirmed_;  // 物品识别确认（单航点有效）
    bool board_confirmed_;   // 板子识别确认（单航点有效）
    bool all_waypoints_tried_;  // 4和5号点均已尝试
    geometry_msgs::PoseStamped target_board_pose_;  // 板子位置
    bool video_node_started_;  // 标记video.py是否已启动

    // --- 新增用于360°旋转角度累计的成员 ---
    bool scan_started_ = false;
    double yaw_start_ = 0.0;
    double last_yaw_ = 0.0;
    double total_rotated_ = 0.0;

    // --- 日志打印控制 ---
    bool init_info_printed_ = false;
    bool waypoint_info_printed_ = false;

public:
    TaskExecutor() : 
        move_base_client_("move_base", true),
        tf_listener_(tf_buffer_),
        current_state_(State::INIT),
        object_confirmed_(false),
        board_confirmed_(false),
        all_waypoints_tried_(false),
        current_se_nav_waypoint_(0),
        video_node_started_(false)
    {
        // 初始化订阅者
        qr_task_sub_ = nh_.subscribe("/task_status/category", 10, &TaskExecutor::taskCallback, this);
        yolo_objects_sub_ = nh_.subscribe("/yolo_objects", 10, &TaskExecutor::objectCallback, this);
        board_qr_sub_ = nh_.subscribe("/board_qrcode", 10, &TaskExecutor::boardQrCallback, this);
        current_waypoint_sub_ = nh_.subscribe("/current_waypoint", 10, &TaskExecutor::currentWaypointCallback, this);
        
        // 初始化发布者
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        yolo_active_pub_ = nh_.advertise<std_msgs::Bool>("/yolo_active", 10);
        skip_waypoint_pub_ = nh_.advertise<std_msgs::Int32>("/skip_waypoint", 10);

        // 只输出1次
        ROS_INFO("任务执行器初始化完成，等待任务和4/5号航点...");
    }

    // 航点回调（到达4号点自动启动video.py）
    void currentWaypointCallback(const std_msgs::Int32::ConstPtr& msg) {
        current_se_nav_waypoint_ = msg->data;
        ROS_INFO("到达航点: %d", current_se_nav_waypoint_);

        // 到达4号点且未启动video.py时，自动启动
        if (current_se_nav_waypoint_ == 4 && !video_node_started_) {
            std::string start_cmd = "rosrun ucar_object_detection video.py &";
            int result = std::system(start_cmd.c_str());
            if (result == 0) {
                ROS_INFO("自动启动video.py成功");
                video_node_started_ = true;
            } else {
                ROS_ERROR("自动启动video.py失败，请检查包名和脚本路径");
                current_state_ = State::FAILED;
                return;
            }
        }

        // 仅在4/5号点且未尝试完所有航点时启动扫描
        if (current_state_ == State::WAIT_FOR_WAYPOINT && !all_waypoints_tried_ &&
            (current_se_nav_waypoint_ == 4 || current_se_nav_waypoint_ == 5)) {
            current_waypoint_ = std::to_string(current_se_nav_waypoint_);
            ROS_INFO("到达%s号点，启动360度识别（物品+板子）", current_waypoint_.c_str());
            current_state_ = State::SCAN_FOR_OBJECT;
            resetScanCache();  // 重置当前航点的识别缓存
            scan_start_time_ = ros::Time::now();
            scan_started_ = false;        // 旋转复位
            total_rotated_ = 0.0;         // 旋转复位

            // 激活YOLO
            std_msgs::Bool active_msg;
            active_msg.data = true;
            yolo_active_pub_.publish(active_msg);
        }
    }

    // 任务类型回调
    void taskCallback(const std_msgs::String::ConstPtr& msg) {
        if (current_state_ != State::INIT) return;

        current_task_ = msg->data;
        ROS_INFO("收到任务类型: %s", current_task_.c_str());

        auto it = TASK_INFO.find(current_task_);
        if (it == TASK_INFO.end()) {
            ROS_ERROR("未知任务类型");
            current_state_ = State::FAILED;
            return;
        }
        target_objects_ = it->second.first;
        target_board_qr_ = it->second.second;
        current_state_ = State::WAIT_FOR_WAYPOINT;
        waypoint_info_printed_ = false; // 允许打印
        ROS_INFO("任务初始化完成，等待到达4号或5号航点");
    }

    // YOLO识别结果回调
    void objectCallback(const std_msgs::String::ConstPtr& msg) {
        if (current_state_ != State::SCAN_FOR_OBJECT) return;
        std::string obj_str = msg->data;
        std::vector<std::string> objects = splitString(obj_str, ",");
        for (auto& obj : objects) {
            obj = correctObjectLabel(obj);
            recent_objects_.push_back(obj);
        }
        if (recent_objects_.size() > 10) recent_objects_.pop_front();
        checkObjectConfirmation();
    }

    // 板子二维码回调
    void boardQrCallback(const std_msgs::String::ConstPtr& msg) {
        if (current_state_ != State::SCAN_FOR_BOARD) return;
        std::string board_qr = msg->data;
        recent_boards_.push_back(board_qr);
        if (recent_boards_.size() > 5) recent_boards_.pop_front();
        checkBoardConfirmation();
    }

    // 物品标签纠正
    std::string correctObjectLabel(const std::string& raw) {
        static const std::unordered_map<std::string, std::string> correction = {
            {"milkdata", "milk"}, {"cokedata", "coke"}, {"whatermelondata", "watermelon"},
            {"watermelondata", "watermelon"}, {"pepperdata", "pepper"}, {"tomatodata", "tomato"},
            {"potatodata", "potato"}, {"appledata", "apple"}, {"bananadata", "banana"}, {"cakedata", "cake"}
        };
        auto it = correction.find(raw);
        return (it != correction.end()) ? it->second : raw;
    }

    // 检查物品确认
    void checkObjectConfirmation() {
        int count = 0;
        for (const auto& obj : recent_objects_) {
            if (std::find(target_objects_.begin(), target_objects_.end(), obj) != target_objects_.end()) {
                count++;
            }
        }
        if (count >= OBJECT_CONFIRM_THRESH) {
            object_confirmed_ = true;
            ROS_INFO("在%s号点确认目标物品", current_waypoint_.c_str());
        }
    }

    // 检查板子确认
    void checkBoardConfirmation() {
        int count = 0;
        for (const auto& qr : recent_boards_) {
            if (qr == target_board_qr_) {
                count++;
            }
        }
        if (count >= BOARD_CONFIRM_THRESH) {
            board_confirmed_ = true;
            try {
                auto transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
                target_board_pose_.header.frame_id = "map";
                target_board_pose_.pose.position.x = transform.transform.translation.x;
                target_board_pose_.pose.position.y = transform.transform.translation.y + 0.5;
                target_board_pose_.pose.orientation = transform.transform.rotation;
                ROS_INFO("在%s号点确认目标板子位置", current_waypoint_.c_str());
            } catch (tf2::TransformException& ex) {
                ROS_WARN("获取板子位置失败: %s", ex.what());
                board_confirmed_ = false;
            }
        }
    }

    // 发送导航目标
    bool sendNavGoal(double x, double y, double yaw) {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        goal.target_pose.pose.orientation = tf2::toMsg(q);

        move_base_client_.sendGoal(goal);
        ROS_INFO("发送导航目标: (%.2f, %.2f, 偏航: %.2f)", x, y, yaw);

        if (!move_base_client_.waitForResult(ros::Duration(30.0))) {
            ROS_WARN("导航超时");
            return false;
        }

        return move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    }

    // 辅助：角度归一化到[-pi, pi]
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

    // 360度扫描，适配麦轮底盘（核心改动）
    void executeScan() {
        // 获取当前yaw
        double yaw_cur = 0.0;
        try {
            auto transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
            tf2::Quaternion q;
            tf2::fromMsg(transform.transform.rotation, q);
            tf2::Matrix3x3 m(q);
            double roll, pitch;
            m.getRPY(roll, pitch, yaw_cur);
        } catch (tf2::TransformException& ex) {
            ROS_WARN("旋转获取朝向失败: %s", ex.what());
            return;
        }

        if (!scan_started_) {
            yaw_start_ = yaw_cur;
            last_yaw_ = yaw_cur;
            total_rotated_ = 0.0;
            scan_start_time_ = ros::Time::now();
            scan_started_ = true;
            ROS_INFO("开始原地旋转，初始角度：%.2f", yaw_start_);
        }

        double dyaw = normalizeAngle(yaw_cur - last_yaw_);
        total_rotated_ += fabs(dyaw);
        last_yaw_ = yaw_cur;

        // 发布旋转指令
        geometry_msgs::Twist twist;
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.angular.z = ROTATE_SPEED;
        cmd_vel_pub_.publish(twist);

        ros::Duration elapsed = ros::Time::now() - scan_start_time_;

        // 停止条件：累计旋转大于等于360° 或 超时
        if (total_rotated_ >= FULL_ROTATION || elapsed.toSec() > SCAN_TIMEOUT) {
            twist.angular.z = 0.0;
            cmd_vel_pub_.publish(twist);
            scan_started_ = false;  // 复位
            ROS_INFO("%s号点扫描完成，实际旋转角度: %.2f弧度", current_waypoint_.c_str(), total_rotated_);

            // 停用YOLO
            std_msgs::Bool active_msg;
            active_msg.data = false;
            yolo_active_pub_.publish(active_msg);

            // 1. 处理物品扫描结果
            if (current_state_ == State::SCAN_FOR_OBJECT) {
                if (object_confirmed_) {
                    ROS_INFO("%s号点物品识别成功，开始扫描板子", current_waypoint_.c_str());
                    current_state_ = State::SCAN_FOR_BOARD;
                    resetScanCache(false);
                    // 重新激活YOLO扫板子
                    scan_start_time_ = ros::Time::now();
                    total_rotated_ = 0.0;
                    scan_started_ = false;
                    active_msg.data = true;
                    yolo_active_pub_.publish(active_msg);
                } else {
                    ROS_WARN("%s号点未识别到目标物品", current_waypoint_.c_str());
                    if (current_waypoint_ == "4") {
                        ROS_INFO("前往5号点重试");
                        current_state_ = State::WAIT_FOR_WAYPOINT;
                    } else {
                        ROS_ERROR("5号点也未识别到物品，任务失败");
                        current_state_ = State::FAILED;
                    }
                }
            }
            // 2. 处理板子扫描结果
            else if (current_state_ == State::SCAN_FOR_BOARD) {
                if (board_confirmed_) {
                    current_state_ = State::NAV_TO_BOARD;
                    // 4号点成功则跳过5号点
                    if (current_waypoint_ == "4" && object_confirmed_) {
                        std_msgs::Int32 skip_msg;
                        skip_msg.data = 5;
                        skip_waypoint_pub_.publish(skip_msg);
                        ROS_INFO("4号点识别完成，已发布跳过5号点信号");
                    }
                } else {
                    if (current_waypoint_ == "4") {
                        ROS_WARN("4号点未识别到板子，前往5号点重试");
                        all_waypoints_tried_ = false;
                        current_state_ = State::WAIT_FOR_WAYPOINT;
                    } else {
                        ROS_ERROR("5号点也未识别到板子，任务失败");
                        current_state_ = State::FAILED;
                    }
                }
            }
        }
    }

    // 重置扫描缓存
    void resetScanCache(bool reset_object = true) {
        if (reset_object) {
            recent_objects_.clear();
            object_confirmed_ = false;
        }
        recent_boards_.clear();
        board_confirmed_ = false;
        scan_started_ = false;   // 每次进扫描前都复位旋转计数
        total_rotated_ = 0.0;
    }

    // 检查是否在板子区域内
    bool isInBoardRegion() {
        try {
            auto transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
            double dx = transform.transform.translation.x - target_board_pose_.pose.position.x;
            double dy = transform.transform.translation.y - target_board_pose_.pose.position.y;

            return fabs(dx) < REGION_SIZE/2 && fabs(dy) < REGION_SIZE/2;
        } catch (tf2::TransformException& ex) {
            ROS_WARN("区域检查失败: %s", ex.what());
            return false;
        }
    }

    // 区域内微调
    void adjustInRegion() {
        try {
            auto transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
            double dx = target_board_pose_.pose.position.x - transform.transform.translation.x;
            double dy = target_board_pose_.pose.position.y - transform.transform.translation.y;

            tf2::Quaternion q_current;
            tf2::fromMsg(transform.transform.rotation, q_current);
            tf2::Matrix3x3 m_current(q_current);
            double current_yaw, r, p;
            m_current.getRPY(r, p, current_yaw);

            tf2::Quaternion q_target;
            tf2::fromMsg(target_board_pose_.pose.orientation, q_target);
            tf2::Matrix3x3 m_target(q_target);
            double target_yaw, r2, p2;
            m_target.getRPY(r2, p2, target_yaw);

            if (fabs(dx) < POSITION_TOLERANCE && fabs(dy) < POSITION_TOLERANCE &&
                fabs(current_yaw - target_yaw) < ANGLE_TOLERANCE) {
                geometry_msgs::Twist twist;
                cmd_vel_pub_.publish(twist);
                ROS_INFO("区域调整完成，准备播报");
                current_state_ = State::PLAY_AUDIO;
                return;
            }

            geometry_msgs::Twist twist;
            twist.linear.x = std::min(0.2, fabs(dx)) * (dx > 0 ? 1 : -1);
            twist.linear.y = std::min(0.2, fabs(dy)) * (dy > 0 ? 1 : -1);
            twist.angular.z = std::min(0.3, fabs(current_yaw - target_yaw)) * ((current_yaw - target_yaw) > 0 ? -1 : 1);
            cmd_vel_pub_.publish(twist);
        } catch (tf2::TransformException& ex) {
            ROS_WARN("微调失败: %s", ex.what());
        }
    }

    // 播放音频播报
    void playAudio() {
        std::string audio_path = "/home/iflytek/ucar_ws/src/speech_command/audio/all/get_" + current_task_ + ".wav";
        std::string cmd = "aplay " + audio_path;
        int result = std::system(cmd.c_str());

        if (result == 0) {
            ROS_INFO("语音播报成功: %s", audio_path.c_str());
            current_state_ = State::COMPLETE;
        } else {
            ROS_WARN("语音播报失败");
            current_state_ = State::FAILED;
        }
    }

    // 状态机主循环
    void run() {
        ros::Rate rate(10);
        while (ros::ok() && current_state_ != State::COMPLETE && current_state_ != State::FAILED) {
            switch (current_state_) {
                case State::INIT: {
                    if (move_base_client_.waitForServer(ros::Duration(1.0))) {
                        if (!init_info_printed_) {
                            ROS_INFO("等待任务类型...");
                            init_info_printed_ = true;
                        }
                    } else {
                        if (!init_info_printed_) {
                            ROS_WARN("等待move_base服务...");
                            init_info_printed_ = true;
                        }
                    }
                    break;
                }

                case State::WAIT_FOR_WAYPOINT: {
                    if (all_waypoints_tried_) {
                        static bool tried_info_printed = false;
                        if (!tried_info_printed) {
                            ROS_INFO("所有航点均已尝试，任务失败");
                            tried_info_printed = true;
                        }
                        current_state_ = State::FAILED;
                    } else {
                        if (!waypoint_info_printed_) {
                            ROS_INFO("等待到达4号或5号航点...");
                            waypoint_info_printed_ = true;
                        }
                    }
                    break;
                }

                case State::SCAN_FOR_OBJECT:
                case State::SCAN_FOR_BOARD: {
                    executeScan();
                    break;
                }

                case State::NAV_TO_BOARD: {
                    tf2::Quaternion q;
                    tf2::fromMsg(target_board_pose_.pose.orientation, q);
                    tf2::Matrix3x3 m(q);
                    double roll, pitch, yaw;
                    m.getRPY(roll, pitch, yaw);

                    if (sendNavGoal(target_board_pose_.pose.position.x, target_board_pose_.pose.position.y, yaw)) {
                        current_state_ = State::ADJUST_IN_REGION;
                    } else {
                        current_state_ = State::FAILED;
                    }
                    break;
                }

                case State::ADJUST_IN_REGION: {
                    if (isInBoardRegion()) {
                        adjustInRegion();
                    } else {
                        tf2::Quaternion q;
                        tf2::fromMsg(target_board_pose_.pose.orientation, q);
                        tf2::Matrix3x3 m(q);
                        double roll, pitch, yaw;
                        m.getRPY(roll, pitch, yaw);
                        sendNavGoal(target_board_pose_.pose.position.x, target_board_pose_.pose.position.y, yaw);
                    }
                    break;
                }

                case State::PLAY_AUDIO: {
                    playAudio();
                    break;
                }

                default: {
                    ROS_WARN("未知状态");
                    current_state_ = State::FAILED;
                    break;
                }
            }

            ros::spinOnce();
            rate.sleep();
        }

        // 任务结束时停用YOLO
        std_msgs::Bool active_msg;
        active_msg.data = false;
        yolo_active_pub_.publish(active_msg);

        if (current_state_ == State::COMPLETE) {
            ROS_INFO("任务成功完成");
        } else {
            ROS_ERROR("任务失败");
        }
    }

    // 辅助函数：向量转字符串
    std::string vecToString(const std::vector<std::string>& vec) {
        std::string res = "[";
        for (size_t i = 0; i < vec.size(); ++i) {
            res += vec[i];
            if (i != vec.size() - 1) res += ", ";
        }
        res += "]";
        return res;
    }

    // 辅助函数：字符串分割
    std::vector<std::string> splitString(const std::string& s, const std::string& delim) {
        std::vector<std::string> res;
        size_t pos = 0;
        std::string temp = s;
        while ((pos = temp.find(delim)) != std::string::npos) {
            res.push_back(temp.substr(0, pos));
            temp.erase(0, pos + delim.size());
        }
        if (!temp.empty()) res.push_back(temp);
        return res;
    }
};

int main(int argc, char**argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "task_executor");
    TaskExecutor executor;
    executor.run();
    return 0;
}
