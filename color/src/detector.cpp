#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <string>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <thread>
#include <chrono>

cv::Mat current_image_buffer;
std::mutex image_mutex;
std::condition_variable image_cv;
bool new_image_available = false;

std::atomic<bool> detection_enabled(false);
std::atomic<bool> detected(false);
std::string detected_result;
ros::Publisher tl_result_pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        std::lock_guard<std::mutex> lock(image_mutex);
        current_image_buffer = cv_ptr->image.clone();
        new_image_available = true;
        image_cv.notify_one();
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void StartDetectCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        detection_enabled = true;
        detected = false;
        ROS_INFO("[detector] 收到开始检测信号，准备检测...");
    }
}

std::string detectTrafficLightColor(const cv::Mat& roi_image) {
    if (roi_image.empty()) {
        ROS_WARN("[detectTrafficLightColor] ROI image is empty. Returning 'Unknown'.");
        return "Unknown";
    }
    if (roi_image.channels() < 3) {
        ROS_ERROR("[detectTrafficLightColor] ROI image does not have 3 channels (BGR). Cannot convert to HSV.");
        return "Unknown";
    }

    cv::Mat hsv_image;
    try {
        cv::cvtColor(roi_image, hsv_image, cv::COLOR_BGR2HSV);
    } catch (const cv::Exception& e) {
        ROS_ERROR("[detectTrafficLightColor] Error converting to HSV: %s", e.what());
        return "Unknown";
    }

    cv::Scalar lower_red1(0, 100, 100);
    cv::Scalar upper_red1(10, 255, 255);
    cv::Scalar lower_red2(160, 100, 100);
    cv::Scalar upper_red2(180, 255, 255);

    cv::Scalar lower_green(35, 100, 100);
    cv::Scalar upper_green(85, 255, 255);

    cv::Mat mask_red1, mask_red2, mask_red;
    cv::inRange(hsv_image, lower_red1, upper_red1, mask_red1);
    cv::inRange(hsv_image, lower_red2, upper_red2, mask_red2);
    cv::bitwise_or(mask_red1, mask_red2, mask_red);

    cv::Mat mask_green;
    cv::inRange(hsv_image, lower_green, upper_green, mask_green);

    int red_pixels = cv::countNonZero(mask_red);
    int green_pixels = cv::countNonZero(mask_green);

    if (red_pixels > green_pixels && red_pixels > 500) {
        return "Red";
    } else if (green_pixels > red_pixels && green_pixels > 500) {
        return "Green";
    } else {
        return "Unknown";
    }
}

void detectionThreadFunc() {
    cv::namedWindow("Traffic Light View", cv::WINDOW_AUTOSIZE);
    while (ros::ok()) {
        if (!detection_enabled) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        ROS_INFO("[detector] 检测模式开启，等待新图片...");
        cv::Mat img;
        {
            std::unique_lock<std::mutex> lock(image_mutex);
            image_cv.wait(lock, [] { return new_image_available || !ros::ok(); });
            if (!ros::ok()) break;
            img = current_image_buffer.clone();
            new_image_available = false;
        }

        if (img.empty()) {
            ROS_WARN("收到空图片，检测失败");
            continue;
        }

        int img_width = img.cols;
        int img_height = img.rows;
        int roi_width = img_width * 0.30;
        int roi_height = img_height * 0.55;
        int roi_x = img_width / 2 - roi_width / 2;
        int roi_y = img_height / 2 - roi_height / 2 - img_height * 0.25;
        roi_x = std::max(0, roi_x);
        roi_y = std::max(0, roi_y);
        if (roi_x + roi_width > img_width) roi_width = img_width - roi_x;
        if (roi_y + roi_height > img_height) roi_height = img_height - roi_y;
        cv::Rect roi(roi_x, roi_y, roi_width, roi_height);

        cv::Mat traffic_light_roi = img(roi);
        std::string color = detectTrafficLightColor(traffic_light_roi);

        cv::rectangle(img, roi, cv::Scalar(255, 0, 0), 2);
        cv::putText(img, "TL: " + color, cv::Point(50, 50),
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        cv::imshow("Traffic Light View", img);
        cv::waitKey(1);

        if (color != "Unknown") {
            std_msgs::String result_msg;
            result_msg.data = color;
            tl_result_pub.publish(result_msg);

            detected_result = color;
            detected = true;
            detection_enabled = false;
            ROS_INFO("[detector] 红绿灯识别完成，结果: %s", color.c_str());
        } else {
            ROS_INFO("[detector] 未能识别红绿灯颜色，等待下一帧...");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    cv::destroyWindow("Traffic Light View");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "detector_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    image_transport::Subscriber image_sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);
    tl_result_pub = nh.advertise<std_msgs::String>("/traffic_light_feedback", 1);
    ros::Subscriber start_detect_sub = nh.subscribe("/start_traffic_light_detection", 1, StartDetectCallback);

    ROS_INFO("[detector] 等待交通灯检测任务...");

    std::thread detect_thread(detectionThreadFunc);

    ros::spin();

    detect_thread.join();

    ROS_INFO("[detector] 节点关闭。");
    return 0;
}
