#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<geometry_msgs/Twist.h>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/opencv.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/imgproc/types_c.h>
#include<opencv2/core/core.hpp>
int record=0;
std::string str;
double twist_linear_x , twist_angular_z;
sensor_msgs::Image hsv_image ;
sensor_msgs::Image hsv_image_point ;
int flag=0;
int cx=0;
int cy=0;
int cz=0;
int ct=0;
int num=0;
int v=0;
ros::Publisher image_pub2;
cv_bridge::CvImage binary_msg;
// std::vector <int> count;
 cv::Mat gray_image;
 ros::Publisher cmd_pub;
cv::Mat binary_image;
 std::vector<std::vector<cv::Point>> contours;
void image_Callback(const sensor_msgs::Image::ConstPtr& msg);

int main(int argc, char **argv){
    ros::init(argc, argv, "follower_line");
    ros::NodeHandle nh;
    ros::NodeHandle nh2;

    //ros::Subscriber img_sub = nh.subscribe("/qingzhou/camera_link/image_raw", 10, image_Callback);
     ros::Subscriber img_sub = nh.subscribe("/usb_cam/image_raw", 10, image_Callback);
    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("/image_hsv",10);
     ros::Publisher img_pub2 = nh.advertise<sensor_msgs::Image>("/image_hsv_point",10);
   //ros::Publisher image_pub2=nh2.advertise<sensor_msgs::Image>("/binary_image",10);
    while(ros::ok()){
        geometry_msgs::Twist twist;
        twist.linear.x = twist_linear_x;
        twist.angular.z = twist_angular_z;
        cmd_pub.publish(twist);
        img_pub.publish(hsv_image);
        img_pub2.publish(hsv_image_point);
        ros::spinOnce();
    }
    return 0;
}


void image_Callback(const sensor_msgs::Image::ConstPtr& msg){

  
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    cv::Mat image = cv_ptr -> image;
    cv::Mat res = image.clone();
    cv::Mat hsv = image.clone();
    cv::Mat test=image.clone();
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 0, 200), cv::Scalar(170, 30, 255), res); 

    binary_image=res.clone();
    
   
    int h = image.rows;
    int w = image.cols;
    int search_top = 5*h/8; int search_bot = search_top+20;

    for(int i = 0; i < search_top; i ++){
        for(int j = 0; j < w; j ++){
            res.at<uchar>(i,j) = 0;
        }
    }
    for(int i = search_bot; i < h; i++){
        for(int j = 0; j < w; j ++){
            res.at<uchar>(i,j) = 0;
        }
    }

    cv::Moments M = cv::moments(res);
    geometry_msgs::Twist twist;
    double start_time = ros::Time::now().toSec();
    ros::Rate loop_rate(1);
    if(M.m00 > 0){
        cx = int (cvRound(M.m10 / M.m00));
        cy = int (cvRound(M.m01 / M.m00));
         
        start_time = ros::Time::now().toSec();
        ROS_INFO("cx: %d cy: %d  ", cx, cy);

        cv::circle(image, cv::Point(cx, cy), 10, (0, 0, 255));
        v = cx - w / 2;
       
        twist_angular_z = float(v) / 300 * 0.3;
        twist_linear_x = 0.3;
 
    }
    else{
         ROS_INFO("not found line!");
         twist_angular_z = 0;
        twist_linear_x = 0;
 

    }
    sensor_msgs::ImagePtr hsv_image_ = cv_bridge::CvImage(std_msgs::Header(), "mono8", binary_image).toImageMsg();
    hsv_image = *hsv_image_;
    sensor_msgs::ImagePtr hsv_image_point_ = cv_bridge::CvImage(std_msgs::Header(), "mono8", res).toImageMsg();
    hsv_image_point = *hsv_image_point_;
        

}
