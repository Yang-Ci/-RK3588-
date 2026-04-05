#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

// 定义状态枚举
enum RobotState {
    FORWARD,          // 正常前进
    MOVE_RIGHT,       // 向右移动
    MOVE_FORWARD_AFTER_RIGHT,  // 右移后前进
    MOVE_LEFT,        // 向左移动
    RETURN_TO_PATH    // 返回原路径后继续前进
};

class ObstacleAvoider {
private:
    ros::NodeHandle nh_;
    ros::Subscriber laser_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Timer state_timer_;
    
    RobotState current_state_;
    float obstacle_distance_;  // 障碍物距离
    float move_distance_;      // 记录当前状态下移动的距离
    ros::Time state_start_time_;  // 状态开始时间
    
    // 配置参数
    const float OBSTACLE_THRESHOLD = 0.25;  // 30cm，障碍物阈值
    const float MOVE_DISTANCE = 0.6;       // 30cm，横向移动距离
    const float FORWARD_DISTANCE = 0.58;    // 60cm，前进距离
    const float LINEAR_SPEED = 0.3;        // 线速度 (m/s)
    
public:
    ObstacleAvoider() : current_state_(FORWARD), obstacle_distance_(10.0), move_distance_(0.0) {
        // 订阅激光雷达数据
        laser_sub_ = nh_.subscribe("/scan", 10, &ObstacleAvoider::laserCallback, this);
        
        // 发布速度指令
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        
        // 状态机定时器，10Hz
        state_timer_ = nh_.createTimer(ros::Duration(0.1), &ObstacleAvoider::stateCallback, this);
    }
    
    // 激光雷达回调函数，检测前方障碍物
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        // 只检测正前方±10度范围内的障碍物
        int center_index = msg->ranges.size() / 2;
        int range = msg->ranges.size() / 18;  // 约10度范围
        
        obstacle_distance_ = 10.0;  // 初始化为一个较大值
        
        // 寻找最近的障碍物
        for (int i = center_index - range; i < center_index + range; ++i) {
            if (msg->ranges[i] < obstacle_distance_ && !isnan(msg->ranges[i])) {
                obstacle_distance_ = msg->ranges[i];
            }
        }
    }
    
    // 状态机回调函数，处理不同状态下的行为
    void stateCallback(const ros::TimerEvent& event) {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0;
        cmd.linear.y = 0;
        cmd.angular.z = 0;
        
        switch (current_state_) {
            case FORWARD:
                ROS_INFO_THROTTLE(1, "正常前进，前方距离: %.2fm", obstacle_distance_);
                
                // 如果检测到障碍物距离小于阈值，切换到避障状态
                if (obstacle_distance_ < OBSTACLE_THRESHOLD) {
                    ROS_INFO("检测到前方障碍物，开始避障");
                    current_state_ = MOVE_RIGHT;
                    state_start_time_ = ros::Time::now();
                    move_distance_ = 0.0;
                } else {
                    // 正常前进
                    cmd.linear.x = LINEAR_SPEED;
                }
                break;
                
            case MOVE_RIGHT:
                ROS_INFO_THROTTLE(1, "向右移动: %.2fm/%.2fm", move_distance_, MOVE_DISTANCE);
                
                // 计算已经移动的距离
                move_distance_ = LINEAR_SPEED * (ros::Time::now() - state_start_time_).toSec();
                
                if (move_distance_ < MOVE_DISTANCE) {
                    // 向右移动 (万向轮的y方向)
                    cmd.linear.y = LINEAR_SPEED;
                } else {
                    // 移动完成，切换到前进状态
                    current_state_ = MOVE_FORWARD_AFTER_RIGHT;
                    state_start_time_ = ros::Time::now();
                    move_distance_ = 0.0;
                    ROS_INFO("向右移动完成，开始前进");
                }
                break;
                
            case MOVE_FORWARD_AFTER_RIGHT:
                ROS_INFO_THROTTLE(1, "前进: %.2fm/%.2fm", move_distance_, FORWARD_DISTANCE);
                
                // 计算已经移动的距离
                move_distance_ = LINEAR_SPEED * (ros::Time::now() - state_start_time_).toSec();
                
                if (move_distance_ < FORWARD_DISTANCE) {
                    // 向前移动
                    cmd.linear.x = LINEAR_SPEED;
                } else {
                    // 前进完成，切换到向左移动状态
                    current_state_ = MOVE_LEFT;
                    state_start_time_ = ros::Time::now();
                    move_distance_ = 0.0;
                    ROS_INFO("前进完成，开始向左移动");
                }
                break;
                
            case MOVE_LEFT:
                ROS_INFO_THROTTLE(1, "向左移动: %.2fm/%.2fm", move_distance_, MOVE_DISTANCE);
                
                // 计算已经移动的距离
                move_distance_ = LINEAR_SPEED * (ros::Time::now() - state_start_time_).toSec();
                
                if (move_distance_ < MOVE_DISTANCE) {
                    // 向左移动 (万向轮的-y方向)
                    cmd.linear.y = -LINEAR_SPEED;
                } else {
                    // 向左移动完成，切换到恢复路径状态
                    current_state_ = RETURN_TO_PATH;
                    ROS_INFO("向左移动完成，恢复原路线前进");
                }
                break;
                
            case RETURN_TO_PATH:
                ROS_INFO_THROTTLE(1, "恢复原路线前进");
                // 保持前进，直到可能遇到下一个障碍物
                cmd.linear.x = LINEAR_SPEED;
                
                // 检查是否远离障碍物，回到正常前进状态
                if (obstacle_distance_ > OBSTACLE_THRESHOLD ) {  // 增加一点余量
                    current_state_ = FORWARD;
                    ROS_INFO("已远离障碍物，回到正常前进模式");
                }
                break;
        }
        
        cmd_vel_pub_.publish(cmd);
    }
};

int main(int argc, char**argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "obstacle_avoider");
    ObstacleAvoider avoider;
    
    try {
        ros::spin();
    } catch (std::exception& e) {
        ROS_ERROR("异常: %s", e.what());
    }
    
    return 0;
}
