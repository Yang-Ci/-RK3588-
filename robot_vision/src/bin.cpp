#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Twist.h>
#include <iostream>
ros::Publisher image_pub;
ros::Publisher cmd_vel_publisher;
void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    try
    {
        // 将ROS图像消息转换为OpenCV图像格式
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
         cv::Mat image = cv_ptr -> image;
        cv::Mat res = image.clone();
        cv::Mat hsv = image.clone();
        // 将彩色图像转换为灰度图像
        cv::Mat gray_image;
        cv::Mat binary_image;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(210, 10, 70), cv::Scalar(230, 20, 85), res); 
        // cv::cvtColor(res, gray_image, cv::COLOR_BGR2GRAY);

        // cv::threshold(gray_image, binary_image, 128, 255, cv::THRESH_BINARY);
        //  cv::cvtColor(cv_ptr->image, res, cv::COLOR_BGR2GRAY);

        // 对灰度图像进行二值化处理
        
        std::vector<std::vector<cv::Point>> contours;
        //cv::threshold(gray_image, binary_image, 55, 255, cv::THRESH_BINARY);
        // cv::findContours(binary_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        // cv::Mat contour_image(binary_image.size(), CV_8UC3, cv::Scalar(0, 0, 0));
        // cv::drawContours(contour_image, contours, -1, cv::Scalar(0, 255, 0), 2);
        //    int cy,deviation_x,deviation_y;
        //    double cx;
        //    double imageCenterX = cv_ptr->image.cols / 2;
        //   double imageCenterY = cv_ptr->image.rows / 2;
        // if (!contours.empty()) {
        //     cv::Moments M = cv::moments(contours[0]);
        //      cx = M.m10 / M.m00;
        //      cy=int(cvRound(M.m01 / M.m00));
        //      deviation_x = cx - imageCenterX;
        //      deviation_y=cy- imageCenterY;
        //      std::cout<<deviation_x<<std::endl;
        //      std::cout<<cx<<std::endl;
        //      std::cout<<imageCenterX<<std::endl;
        //     // 生成控制命令
        //     // 这里假设控制命令为简单的角度偏差
        //     double angle = deviation_x * 0.1; // 偏差乘以某个系数得到角度控制命令

        
        //     geometry_msgs::Twist cmd_vel_msg;
        //     cmd_vel_msg.angular.z=0;
        //     cmd_vel_msg.linear.x=0.3;
        //     cmd_vel_msg.angular.z = angle;
        //     cmd_vel_publisher.publish(cmd_vel_msg);
        // 在这里可以对二值化后的图像进行进一步处理，或者将其用于其他用途
        cv_bridge::CvImage binary_msg;
        binary_msg.header = msg->header;
        binary_msg.encoding = sensor_msgs::image_encodings::MONO8;
        binary_msg.image = binary_image;
        image_pub.publish(binary_msg.toImageMsg());

       
        
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
       
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_binarization_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh2;
    ros::NodeHandle nh3;
    ros::Rate r(1);

    // 订阅图像话题
    ros::Subscriber sub = nh.subscribe<sensor_msgs::Image>("/usb_cam/image_raw", 1, imageCallback);
    image_pub=nh.advertise<sensor_msgs::Image>("binary_image",10);
    //cmd_vel_publisher=nh.advertise<geometry_msgs::Twist>("/qingzhou/ackermann_steering_controller/cmd_vel",10);
    ros::spin();
    return 0;
}