#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <cmath>

// 弧度转角度
double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

int main(int argc, char**argv) {
    // 初始化节点
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "rotate_node");
    ros::NodeHandle nh;

    // 创建发布者，发布速度指令
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // 创建TF监听器，用于获取当前姿态
    tf::TransformListener listener;

    // 设置循环频率
    ros::Rate rate(10.0);

    // 初始姿态
    tf::StampedTransform initial_transform;
    bool got_initial = false;

    // 等待获取初始姿态
    while (nh.ok() && !got_initial) {
        try {
            // 假设底盘坐标系为base_link，世界坐标系为odom
            listener.lookupTransform("odom", "base_link", ros::Time(0), initial_transform);
            got_initial = true;
        } catch (tf::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    // 获取初始角度（yaw角）
    double initial_yaw = tf::getYaw(initial_transform.getRotation());
    double current_yaw = initial_yaw;
    double target_yaw = initial_yaw + 2 * M_PI;  // 目标：旋转360度（2π弧度）

    ROS_INFO("开始旋转，初始角度: %.2f度", rad2deg(initial_yaw));

    // 旋转速度控制消息
    geometry_msgs::Twist twist;
    twist.linear.x = 0.0;    // 不前进
    twist.angular.z = 0.3;   // 旋转角速度，可根据实际情况调整

    // 持续旋转直到达到目标角度
    while (nh.ok()) {
        try {
            tf::StampedTransform transform;
            listener.lookupTransform("odom", "base_link", ros::Time(0), transform);
            current_yaw = tf::getYaw(transform.getRotation());

            // 计算已旋转的角度
            double delta_yaw = current_yaw - initial_yaw;
            // 处理角度环绕问题（-π到π之间）
            if (delta_yaw < 0) delta_yaw += 2 * M_PI;

            ROS_INFO("已旋转: %.2f度 / 360度", rad2deg(delta_yaw));

            // 检查是否已旋转一圈
            if (delta_yaw >= 2 * M_PI - 0.1) {  // 允许小的误差
                twist.angular.z = 0.0;  // 停止旋转
                cmd_vel_pub.publish(twist);
                ROS_INFO("旋转完成！");
                break;
            }

            // 发布速度指令
            cmd_vel_pub.publish(twist);

        } catch (tf::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }

        rate.sleep();
    }

    return 0;
}
