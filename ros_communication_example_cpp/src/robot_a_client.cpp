// robot_a_client.cpp
// 功能：监控本地导航日志，检测完成后调用B机器人服务

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <fstream>
#include <string>
#include <regex>
#include <sys/stat.h>


// 检查日志文件是否包含完成标志
bool checkNavigationCompleted(const std::string& log_file) {
    struct stat file_info;
    if (stat(log_file.c_str(), &file_info) != 0) {
        ROS_WARN("日志文件不存在：%s", log_file.c_str());
        return false;
    }
    std::ifstream log_stream(log_file);
    if (!log_stream.is_open()) {
        ROS_ERROR("无法打开日志文件：%s", log_file.c_str());
        return false;
    }
    std::regex completion_regex("已到达航点4，开始执行仿真任务...");
    std::string line;
    while (std::getline(log_stream, line)) {
        if (std::regex_search(line, completion_regex)) {
            log_stream.close();
            ROS_INFO("日志中检测到导航完成标志！");
            return true;
        }
    }
    log_stream.close();
    return false;
}

int main(int argc, char **argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "robot_a_client");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("/robot_b/start_simulation");
    if (!client.waitForExistence(ros::Duration(5.0))) {
        ROS_ERROR("等待5秒未发现B机器人服务，请先启动robot_b_server！");
        return 1;
    }
    std::string log_file = "/tmp/ucar_nav.log"; // 自己导航进程日志
    ROS_INFO("A客户端启动，监控日志路径：%s", log_file.c_str());
    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(300.0);
    ros::Rate check_rate(1); // 1Hz
    while (ros::ok()) {
        if ((ros::Time::now() - start_time) > timeout) {
            ROS_ERROR("超时%d秒，未检测到导航完成标志！", (int)timeout.toSec());
            return 1;
        }
        if (checkNavigationCompleted(log_file)) {
            ROS_INFO("检测到导航完成，开始调用B机器人...");
            std_srvs::Trigger srv;
            if (client.call(srv)) {
                if (srv.response.success) {
                    ROS_INFO("B机器人响应：%s", srv.response.message.c_str());

                } else {
                    ROS_ERROR("B机器人任务失败：%s", srv.response.message.c_str());
                }
            } else {
                ROS_ERROR("调用B机器人服务失败，请检查B是否在线！");
            }
            break;
        }
        ros::spinOnce();
        check_rate.sleep();
    }
    return 0;
}
