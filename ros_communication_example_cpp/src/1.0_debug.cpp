#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <fstream>

bool move_to_position(const std::string& target_pos) {
    ROS_INFO_STREAM("移动到位置: " << target_pos);
    ros::Duration(3).sleep();
    ROS_INFO_STREAM("已到达指定位置");
    return true;
}

// 辅助函数：使用system命令获取服务列表
std::vector<std::string> getServiceList() {
    std::vector<std::string> services;
    std::system("rosservice list > /tmp/ros_services.txt");
    
    std::ifstream file("/tmp/ros_services.txt");
    std::string line;
    if (file.is_open()) {
        while (std::getline(file, line)) {
            services.push_back(line);
        }
        file.close();
    }
    return services;
}

// 辅助函数：检查服务是否存在
bool serviceExists(const std::string& service_name) {
    auto services = getServiceList();
    for (const auto& s : services) {
        if (s == service_name) return true;
    }
    return false;
}

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "robot_a_client");
    ros::NodeHandle n("~");  // 使用私有命名空间，避免名称冲突
    
    // 打印环境变量
    ROS_INFO("ROS_MASTER_URI: %s", std::getenv("ROS_MASTER_URI"));
    ROS_INFO("ROS_IP: %s", std::getenv("ROS_IP"));
    ROS_INFO("ROS_HOSTNAME: %s", std::getenv("ROS_HOSTNAME"));
    
    // 手动测试ROS Master连接
    ROS_INFO("正在测试与ROS Master的连接...");
    if (!ros::master::check()) {
        ROS_FATAL("无法连接到ROS Master！请检查ROS_MASTER_URI设置");
        return 1;
    }
    ROS_INFO("已成功连接到ROS Master");
    
    // 获取当前所有节点列表
    std::vector<std::string> nodes;
    ros::master::getNodes(nodes);
    ROS_INFO("发现 %zu 个活跃节点:", nodes.size());
    for (const auto& node : nodes) {
        ROS_INFO("  - %s", node.c_str());
    }
    
    std::vector<std::string> positions = {"位置1", "位置2", "位置3"};
    
    for (const auto& pos : positions) {
        if (!move_to_position(pos)) {
            ROS_ERROR("移动失败，跳过当前位置");
            continue;
        }
        
        // 增强服务等待逻辑
        ROS_INFO("等待服务 /start_task 可用...");
        bool service_available = false;
        int max_attempts = 5;
        
        for (int attempt = 1; attempt <= max_attempts; ++attempt) {
            service_available = ros::service::waitForService("/start_task", ros::Duration(2.0));
            
            if (service_available) {
                ROS_INFO("服务 /start_task 可用，尝试调用 (尝试 %d/%d)", attempt, max_attempts);
                
                // 检查服务是否存在
                bool service_exists = serviceExists("/start_task");
                ROS_INFO("服务 /start_task 存在: %s", service_exists ? "是" : "否");
                
                break;
            }
            
            // 尝试手动刷新服务列表
            ROS_WARN("服务 /start_task 不可用，刷新服务列表并重试 (尝试 %d/%d)...", attempt, max_attempts);
            ros::Duration(1.0).sleep();
        }
        
        if (!service_available) {
            ROS_ERROR("服务 /start_task 在多次尝试后仍不可用！");
            
            // 打印服务列表
            auto services = getServiceList();
            ROS_INFO("发现 %zu 个活跃服务:", services.size());
            for (const auto& service : services) {
                ROS_INFO("  - %s", service.c_str());
            }
            
            continue;
        }
        
        // 创建服务客户端并调用服务
        ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/start_task");
        std_srvs::Empty srv;
        
        if (client.call(srv)) {
            ROS_INFO("服务调用成功！正在等待机器人B完成任务...");
            ros::Duration(5.0).sleep();
        } else {
            ROS_ERROR("服务调用失败！错误信息: %s", client.getService().c_str());
            
            // 检查客户端状态
            ROS_INFO("客户端状态: 服务存在=%d", client.exists());
        }
    }
    
    return 0;
}