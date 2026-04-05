#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <std_msgs/Bool.h>

// 定义小车的状态
enum class CarState {
    LANE_FOLLOWING,
    OBSTACLE_DETECTED,
    OBSTACLE_AVOIDANCE_LEFT,
    OBSTACLE_AVOIDANCE_FORWARD,
    OBSTACLE_AVOIDANCE_RIGHT,
    REACQUIRING_LANE,
    FINISHED,
    PAUSED
};

class LineFollower {
public:
    LineFollower();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

private:
    // ROS 节点、发布者和订阅者
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber start_signal_sub_;
    ros::Publisher cmd_vel_pub_;
    image_transport::Publisher debug_image_pub_;

    // 当前状态
    CarState state_;
    bool is_active_;
    
    // PID 控制器参数
    double Kp_, Ki_, Kd_;
    double error_, last_error_, integral_error_;

    // 速度参数
    double linear_velocity_;
    double angular_velocity_max_;
    
    // 障碍物检测参数
    int obstacle_pixel_threshold_;
    int reacquire_success_threshold_ = 5; // 连续5帧成功检测到线才认为重新获得
    int reacquire_current_count_ = 0; // 当前连续成功检测计数

    // 避障动作参数
    ros::Time avoidance_start_time_;
    ros::Duration avoidance_turn_duration_;
    ros::Duration avoidance_forward_duration_;

    ros::Time search_turn_start_time_;
    ros::Duration search_turn_duration_segment_; // 每个小段转弯的时间
    double current_search_angular_z_; // 当前搜索的角速度方向

    double current_curve_indicator_; // 指示赛道弯曲程度，例如底部和中部质心的偏差

    // 函数
    void processImage(const cv::Mat& input_image, cv::Mat& output_image, int& error, bool& obstacle_detected);
    geometry_msgs::Twist calculateControlSignal(int error);
    void startSignalCallback(const std_msgs::Bool::ConstPtr& msg);
};

LineFollower::LineFollower() : it_(nh_) {
    // 初始化ROS句柄、发布者和订阅者
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &LineFollower::imageCallback, this);
    //start_signal_sub_ = nh_.subscribe("/start_signal",1, &LineFollower::startSignalCallback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    debug_image_pub_ = it_.advertise("/line_follower/debug_image", 1);

    // 从参数服务器加载参数 (这些值需要你根据实际情况调试)
    nh_.param("pid/kp", Kp_, 0.4);
    nh_.param("pid/ki", Ki_, 0.0);
    nh_.param("pid/kd", Kd_, 0.1);
    ROS_WARN("Kp_ = %f", Kp_);
    ROS_WARN("Ki_ = %f", Ki_);
    ROS_WARN("Kd_ = %f", Kd_);
    nh_.param("speed/linear", linear_velocity_, 0.2); // m/s
    ROS_WARN("linear_velocity = %f", linear_velocity_);
    nh_.param("speed/angular_max", angular_velocity_max_, 1.0); // rad/s
    ROS_WARN("angular_max = %f", angular_velocity_max_);
    nh_.param("obstacle/pixel_threshold", obstacle_pixel_threshold_, 2000);
    
    double turn_duration_sec, forward_duration_sec;
    nh_.param("avoidance/turn_duration", turn_duration_sec, 1.5); // 假设转弯1.5秒
    nh_.param("avoidance/forward_duration", forward_duration_sec, 2.0); // 假设直行2秒
    avoidance_turn_duration_ = ros::Duration(turn_duration_sec);
    avoidance_forward_duration_ = ros::Duration(forward_duration_sec);

    // 初始化PID和状态变量
    error_ = 0.0;
    last_error_ = 0.0;
    integral_error_ = 0.0;
    state_ = CarState::PAUSED;
    is_active_ = false;

    search_turn_duration_segment_ = ros::Duration(0.7); // 例如，每0.7秒切换一次转向方向
    current_search_angular_z_ = angular_velocity_max_ * 0.7; // 默认先向右转

}

void LineFollower::startSignalCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data && !is_active_) { // 如果收到true且当前未激活
        is_active_ = true;
        state_ = CarState::LANE_FOLLOWING; // 切换到巡线状态
        ROS_INFO("Start signal received! Car activated.");
    } else if (!msg->data && is_active_) { // 如果收到false且当前已激活
        is_active_ = false;
        state_ = CarState::PAUSED; // 切换到暂停状态
        ROS_INFO("Stop signal received! Car paused.");
    }
}

void LineFollower::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    is_active_ = true;
    state_ = CarState::LANE_FOLLOWING; // 切换到巡线状态
    ROS_INFO("Start signal received! Car activated.");

    if (!is_active_) {
        // 如果未激活，保持小车静止
        ROS_WARN("car pause!!!");
        geometry_msgs::Twist stop_cmd;
        stop_cmd.linear.x = 0;
        stop_cmd.angular.z = 0;
        cmd_vel_pub_.publish(stop_cmd);
        return; // 直接返回，不执行后续逻辑
    }

    is_active_ = true;
    state_ = CarState::LANE_FOLLOWING; // 切换到巡线状态
    ROS_INFO("Start signal received! Car activated.");

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat processed_image;
    int current_error = 0;
    bool obstacle_detected = false;
    
    // 1. 图像处理
    processImage(cv_ptr->image, processed_image, current_error, obstacle_detected);
    
    // 发布调试图像
    cv_bridge::CvImage debug_msg;
    debug_msg.header = msg->header;
    debug_msg.encoding = sensor_msgs::image_encodings::MONO8; // 调试图像通常是二值图，所以用MONO8
    debug_msg.image = processed_image;
    debug_image_pub_.publish(debug_msg.toImageMsg());

    // 2. 状态机决策
    geometry_msgs::Twist cmd_vel;
    
    // 检查是否需要从巡线切换到避障状态
    if (state_ == CarState::LANE_FOLLOWING && obstacle_detected) {
        state_ = CarState::OBSTACLE_DETECTED;
        ROS_INFO("State Changed: OBSTACLE_DETECTED");
    }
    
    switch (state_) {
        case CarState::LANE_FOLLOWING:
            cmd_vel = calculateControlSignal(current_error);
            break;

        case CarState::OBSTACLE_DETECTED:
            // 这是一个瞬时状态，立即开始左转并记录时间
            state_ = CarState::OBSTACLE_AVOIDANCE_LEFT;
            avoidance_start_time_ = ros::Time::now();
            cmd_vel.linear.x = linear_velocity_ / 2.0; // 减速
            cmd_vel.angular.z = angular_velocity_max_; // 左转（正值为左）
            break;

        case CarState::OBSTACLE_AVOIDANCE_LEFT:
            cmd_vel.linear.x = linear_velocity_ / 2.0;
            cmd_vel.angular.z = angular_velocity_max_;
            if (ros::Time::now() - avoidance_start_time_ > avoidance_turn_duration_) {
                state_ = CarState::OBSTACLE_AVOIDANCE_FORWARD;
                avoidance_start_time_ = ros::Time::now(); // 重置计时器
            }
            break;
            
        case CarState::OBSTACLE_AVOIDANCE_FORWARD:
            cmd_vel.linear.x = linear_velocity_;
            cmd_vel.angular.z = 0;
            if (ros::Time::now() - avoidance_start_time_ > avoidance_forward_duration_) {
                state_ = CarState::OBSTACLE_AVOIDANCE_RIGHT;
                avoidance_start_time_ = ros::Time::now();
            }
            break;

        case CarState::OBSTACLE_AVOIDANCE_RIGHT:
            cmd_vel.linear.x = linear_velocity_ / 2.0;
            cmd_vel.angular.z = -angular_velocity_max_; // 右转
            if (ros::Time::now() - avoidance_start_time_ > avoidance_turn_duration_) {
                state_ = CarState::REACQUIRING_LANE;
            }
            break;

        case CarState::REACQUIRING_LANE:
            // 重新寻找赛道线
            cmd_vel.linear.x = linear_velocity_ / 2.0;
            // 检查是否需要切换转向方向
            if (ros::Time::now() - search_turn_start_time_ > search_turn_duration_segment_) {
                current_search_angular_z_ *= -1; // 切换方向（例如，从正到负，或从负到正）
                search_turn_start_time_ = ros::Time::now(); // 重置计时
                ROS_INFO("REACQUIRING_LANE: Switched search direction to %f", current_search_angular_z_);
            }
            cmd_vel.angular.z = current_search_angular_z_;
            if (current_error != 999 && std::abs(current_error) < cv_ptr->image.cols / 4) { 
                // 误差在图像1/4宽度内
                reacquire_current_count_++;
            if (reacquire_current_count_ >= reacquire_success_threshold_) {
                state_ = CarState::LANE_FOLLOWING;
                ROS_INFO("State Changed: LANE_FOLLOWING (Reacquired)");
                reacquire_current_count_ = 0; // 重置计数器
                // 找到线后，立即将转向速度设为PID计算值，而不是搜索值
                cmd_vel = calculateControlSignal(current_error);
            }
            } else {
            reacquire_current_count_ = 0; // 如果条件不满足，重置计数器
            }
            break;

        case CarState::FINISHED:
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            break;
        
        case CarState::PAUSED:
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            break;
    }

    // 3. 发布控制指令
    cmd_vel_pub_.publish(cmd_vel);
}

// 图像处理函数
void LineFollower::processImage(const cv::Mat& input_image, cv::Mat& output_mask, int& error, bool& obstacle_detected) {
    // --- 赛道线处理 ---
    // 截取ROI (Region of Interest)，只关心图像下半部分，用于赛道线检测
    // 这样做可以避免上方的干扰，并聚焦于小车前方的赛道
    cv::Rect roi_rect(0, input_image.rows / 4, input_image.cols, input_image.rows *3/4);
    cv::Mat roi = input_image(roi_rect);

    // 转换到HSV颜色空间
    cv::Mat hsv_image;
    cv::cvtColor(roi, hsv_image, cv::COLOR_BGR2HSV);

    // 阈值分割 (这里的白色阈值需要调试，根据实际赛道线的颜色和光照条件调整)
    cv::Scalar lower_white(0, 0, 150);
    cv::Scalar upper_white(180, 50, 255); // 调整V值以更好地识别白色
    cv::Mat raw_hsv_mask; // 临时存储inRange的结果，现在它会包含白线和白字
    cv::inRange(hsv_image, lower_white, upper_white, raw_hsv_mask);

    // --- 形态学操作 (可选，用于连接断裂的线或去除小噪声) ---
    // 先进行闭运算连接线，再开运算去除小噪声
    cv::Mat kernel_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(raw_hsv_mask, raw_hsv_mask, cv::MORPH_CLOSE, kernel_close);

    cv::Mat kernel_open = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(raw_hsv_mask, raw_hsv_mask, cv::MORPH_OPEN, kernel_open);

    // --- 连通区域分析和更严格的筛选 ---
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(raw_hsv_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat filtered_line_mask = cv::Mat::zeros(raw_hsv_mask.size(), raw_hsv_mask.type());

    // 这些常量需要根据你的实际情况仔细测量和调试！
    const double MIN_LINE_AREA = 100.0;   // 最小连通区域面积（排除小噪声点）
    const double MAX_LINE_AREA = roi.rows * roi.cols * 0.7; // 最大连通区域面积（排除整个图像被识别的情况）

    // 赛道线的典型宽度范围（像素）
    // 这可能需要根据赛道实际情况和相机距离调整。
    // 对于白色赛道线，这个宽度应该比较稳定
    const double TYPICAL_LINE_WIDTH_MIN = 15.0; // 赛道线最小像素宽度
    const double TYPICAL_LINE_WIDTH_MAX = 50.0; // 赛道线最大像素宽度

    // 字母的典型尺寸和长宽比（需要观察字母在图像中的实际表现）
    const double MAX_LETTER_AREA = 800.0; // 字母的最大面积，通常比赛道线的一段小很多
    const double MAX_LETTER_DIM = 40.0;   // 字母的最大边长（宽度或高度）
    const double LETTER_ASPECT_RATIO_TOLERANCE = 2.5; // 字母的长宽比通常不会太极端

    for (size_t i = 0; i < contours.size(); i++) {
        cv::Rect bbox = cv::boundingRect(contours[i]);
        double contour_area = cv::contourArea(contours[i]);

        // 1. 基本面积过滤：排除太小（噪声）或太大（整个背景）的区域
        if (contour_area < MIN_LINE_AREA || contour_area > MAX_LINE_AREA) {
            continue;
        }

        // 2. 几何形状过滤：区分线和字母
        double min_dim = std::min(bbox.width, bbox.height); // 连通区域的短边，可以看作“厚度”
        double max_dim = std::max(bbox.width, bbox.height); // 连通区域的长边
        double aspect_ratio = max_dim / min_dim; // 长宽比

        bool is_potential_line = false;

        // 条件 A: 赛道线通常是狭长的（长边显著大于短边），且短边（宽度）在特定范围内
        if (aspect_ratio > 2.0 && // 长宽比大于2，即是长条形
            min_dim >= TYPICAL_LINE_WIDTH_MIN && min_dim <= TYPICAL_LINE_WIDTH_MAX)
        {
            is_potential_line = true;
        }
        // 条件 B: 赛道线也可能是短小一段但宽度符合的（例如在弯道顶点）
        // 允许短小的线段，只要其宽度符合赛道线的宽度
        else if (min_dim >= TYPICAL_LINE_WIDTH_MIN && min_dim <= TYPICAL_LINE_WIDTH_MAX &&
                 max_dim >= TYPICAL_LINE_WIDTH_MIN && max_dim <= TYPICAL_LINE_WIDTH_MAX * 2) // 短边和长边都在合理范围内
        {
             is_potential_line = true;
        }
        // 进一步细化：如果文字非常大，可能会被错误识别为线，所以增加文字排除条件
        // 3. 排除可能是文字的区域：面积小，且形状不那么狭长
        if (!is_potential_line) { // 如果上面判断不是线，再看它是不是字母
            if (contour_area < MAX_LETTER_AREA && // 面积小
                max_dim < MAX_LETTER_DIM &&     // 边长小
                aspect_ratio < LETTER_ASPECT_RATIO_TOLERANCE) // 长宽比不极端 (接近方形或圆形)
            {
                // 这可能是字母，跳过
                continue;
            }
        }
        // 如果通过了所有过滤条件，则认为这是赛道线的一部分
        cv::drawContours(filtered_line_mask, contours, i, cv::Scalar(255), cv::FILLED);
    }

// --- 多段质心计算 ---
    int cx_bottom = -1, cx_middle = -1; // 初始化为无效值

    // 定义多条扫描线的高度（占ROI的高度比例）
    // 这里的行号是相对于ROI的行号
    int scan_line_bottom_roi_y = roi.rows * 0.9; // 靠近ROI底部
    int scan_line_middle_roi_y = roi.rows * 0.5; // ROI中部

    // 扫描底部线
    if (scan_line_bottom_roi_y >= 0 && scan_line_bottom_roi_y < roi.rows) {
        cv::Mat bottom_line_row = filtered_line_mask.row(scan_line_bottom_roi_y);
        cv::Moments M_bottom = cv::moments(bottom_line_row, true);
        if (M_bottom.m00 > 0) {
            cx_bottom = M_bottom.m10 / M_bottom.m00;
        }
    }

    // 扫描中部线
    if (scan_line_middle_roi_y >= 0 && scan_line_middle_roi_y < roi.rows) {
        cv::Mat middle_line_row = filtered_line_mask.row(scan_line_middle_roi_y);
        cv::Moments M_middle = cv::moments(middle_line_row, true);
        if (M_middle.m00 > 0) {
            cx_middle = M_middle.m10 / M_middle.m00;
        }
    }

    // 计算巡线误差 (仍然以底部线为主要依据)
    if (cx_bottom != -1) {
        error = filtered_line_mask.cols / 2 - cx_bottom; // 误差 = 图像中心 - 赛道线中心
        // 更新主要的误差值
    } else {
        error = 999; // 未找到赛道线
        ROS_WARN_THROTTLE(1, "Lane line not found in main ROI!");
    }

    // 计算曲率指标
    current_curve_indicator_ = 0.0; // 默认直线
    if (cx_bottom != -1 && cx_middle != -1) {
        // 底部和中部质心之间的水平距离，反映弯曲程度
        double center_diff = static_cast<double>(cx_bottom - cx_middle);
        // 将差值归一化，例如，除以图像宽度的一半，使其在 -1 到 1 之间
        current_curve_indicator_ = center_diff / (roi.cols / 2.0);
        // 限制在合理范围，防止极端值
        if (current_curve_indicator_ > 1.0) current_curve_indicator_ = 1.0;
        if (current_curve_indicator_ < -1.0) current_curve_indicator_ = -1.0;

        // ROS_INFO_THROTTLE(0.5, "Curve Indicator: %.2f", current_curve_indicator_);
    } else {
        // 如果某条线未找到，则曲率指标设为0，或者上一次的值
        // 这里为了简单，设为0，表示不确定弯道情况，可以考虑保持last_curve_indicator_
    }

    // --- 障碍物处理 ---
    // (假设障碍物是黄色的，需要调试其HSV范围)
    // 可以在完整的图像上检测障碍物，或者在另一个ROI上
    // 这里我们也在ROI上检测，如果障碍物在ROI上方，可能需要调整ROI或者使用完整图像
    // cv::Scalar lower_yellow(26, 43, 46);  // 典型的黄色HSV下限
    // cv::Scalar upper_yellow(34, 255, 255); // 典型的黄色HSV上限
    // cv::Mat obstacle_mask;
    // cv::inRange(hsv_image, lower_yellow, upper_yellow, obstacle_mask);
    
    // // 计算障碍物掩码中非零像素的数量，如果超过阈值则认为检测到障碍物
    // int obstacle_pixels = cv::countNonZero(obstacle_mask);
    // if (obstacle_pixels > obstacle_pixel_threshold_) {
    //     obstacle_detected = true;
    //     ROS_WARN_THROTTLE(1, "Obstacle Detected! Pixels: %d", obstacle_pixels); // 增加一个警告，方便调试
    // } else {
    //     obstacle_detected = false;
    // }

    // --- 计算巡线误差 ---
    // 使用图像矩来找到赛道线的质心
    cv::Moments M = cv::moments(filtered_line_mask, true);
    if (M.m00 > 0) { // 确保有足够的像素点来计算质心
        int cx = M.m10 / M.m00; // 质心的X坐标
        // 误差 = 图像中心 - 赛道线中心。如果中心在左边，误差为正，需要右转；在右边，误差为负，需要左转。
        error = filtered_line_mask.cols / 2 - cx;
    } else {
        // 如果没有找到赛道线，将误差设置为一个特殊值（999），通知上层逻辑
        error = 999; 
        ROS_WARN_THROTTLE(1, "Lane line not found!");
    }

    // 将处理后的赛道线掩码作为调试图像输出
    output_mask = filtered_line_mask; 
}

// PID计算函数
geometry_msgs::Twist LineFollower::calculateControlSignal(int current_error) {
    geometry_msgs::Twist twist_cmd;
    twist_cmd.linear.x = linear_velocity_; // 默认前进速度
    twist_cmd.angular.z = 0; // 默认角速度

    if (current_error == 999) { // 没找到线的情况
        // 如果长时间没找到线，可以考虑停车或者执行搜索策略
        // 这里为了保持运动，简单地使用上一次的误差进行控制，但这不是最优解
        // 更好的做法是进入一个"搜索线"的状态或降低速度
        current_error = last_error_; // 保持上一次的转向趋势
        twist_cmd.linear.x = linear_velocity_ / 2.0; // 降低速度以进行搜索
    }
    
    integral_error_ += current_error;
    // 积分项需要防止过大积累，可以设置积分饱和限制
    // 例如：if (integral_error_ > MAX_INTEGRAL) integral_error_ = MAX_INTEGRAL;
    // 或 if (integral_error_ < MIN_INTEGRAL) integral_error_ = MIN_INTEGRAL;

    double derivative_error = current_error - last_error_;

    // PID控制公式
    double angular_z = Kp_ * current_error + Ki_ * integral_error_ + Kd_ * derivative_error;
    
    last_error_ = current_error; // 更新上次误差

    // 限制最大转向速度，避免小车转向过猛
    twist_cmd.angular.z = std::max(-angular_velocity_max_, std::min(angular_velocity_max_, angular_z));
    
    // 环岛/大弯道降速策略：转向越大，速度越慢，以提高稳定性
    // 这个系数 0.8 可以根据实际效果调整
    twist_cmd.linear.x = linear_velocity_ * (1 - 0.8 * std::abs(twist_cmd.angular.z) / angular_velocity_max_);
    // 确保线速度不会因为转向而降到过低或变为负数
    if (twist_cmd.linear.x < 0.05) twist_cmd.linear.x = 0.05; // 设置一个最小线速度

    return twist_cmd;
}

// main函数
int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "line_follower_node");
    
    // 创建LineFollower类的实例
    LineFollower lf;
    
    // 进入ROS事件循环，等待回调函数被调用
    ros::spin();
    
    return 0;
}
