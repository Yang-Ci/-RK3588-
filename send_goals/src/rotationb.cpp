#include "ros/ros.h"
#include "/home/iflytek/ucar_ws/devel/include/object_information_msgs/Object.h"  // 替换为实际包名
#include <map>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include<iostream>
#include <geometry_msgs/Twist.h>
#include <cmath>
ros::Subscriber subscriber;

bool object_detected = false;
bool isStop = false;
bool isFound = false;
std::string label;

double object_center_x = 0;
const double FULL_ROTATION_TIME = 2 * M_PI / 0.4;
ros::Time rotation_start_time;
ros::Time rotation_start_time_1;
bool rotation_started = false;
bool rotation_started_1 = false;


void callback(const object_information_msgs::Object::ConstPtr& msg)
    {
        
        
            if(label == msg->label){
                object_detected = true;
                object_center_x = msg->size.y/2 + msg->position.position.x; //di
            }
    }
    

 
void controlCar(ros::Publisher& pub) {
    geometry_msgs::Twist msg;
    
    if (object_detected) {
        
        // 根据目标物体中心点和图像中心点的距离来控制旋转
        int error_x = object_center_x - 320;
        
        if (abs(error_x) < 20) {
            // 如果误差小于一定值，则认为小车已经正对目标物体中心
            msg.angular.z = 0.0;
            ROS_INFO("Car aligned with the object.");
            isFound = true;
            isStop = true;
            subscriber.shutdown();
        } else if (error_x > 0) {
            // 目标物体在图像中心右侧，小车应向左转
            msg.angular.z = 0.4;
            ROS_INFO("Turning left.");
            if (!rotation_started_1) {
            // 开始旋转并记录开始时间
            rotation_started_1 = true;
            rotation_start_time_1 = ros::Time::now();
            }
            ros::Duration elapsed_time_1 = ros::Time::now() - rotation_start_time_1;
            if(elapsed_time_1.toSec() < FULL_ROTATION_TIME){
            msg.angular.z = 0.4;
            ROS_INFO("Searching for the object.");
            }else{
                msg.angular.z = 0.0;
                isStop = true;
                ROS_INFO("zhaodao le dan you zhaobudao  Stopping the car.");
            }
        } else {
            // 目标物体在图像中心左侧，小车应向右转
            msg.angular.z = -0.4;
            ROS_INFO("Turning right.");
            if (!rotation_started_1) {
            // 开始旋转并记录开始时间
            rotation_started_1 = true;
            rotation_start_time_1 = ros::Time::now();
            }
            ros::Duration elapsed_time_1 = ros::Time::now() - rotation_start_time_1;
            if(elapsed_time_1.toSec() < FULL_ROTATION_TIME){
            msg.angular.z = -0.4;
            ROS_INFO("Searching for the object.");
            }else{
                msg.angular.z = 0.0;
                isStop = true;
                ROS_INFO("zhaodao le dan you zhaobudao  Stopping the car.");
            }
        }
        rotation_started = false;
    } else {
        if (!rotation_started) {
            // 开始旋转并记录开始时间
            rotation_started = true;
            rotation_start_time = ros::Time::now();
        }

        ros::Duration elapsed_time = ros::Time::now() - rotation_start_time;
        // 如果未检测到目标物体，小车继续旋转
        if(elapsed_time.toSec() < FULL_ROTATION_TIME){
            msg.angular.z = 0.4;
            ROS_INFO("Searching for the object.");
        }else{
            msg.angular.z = 0.0;
            isStop = true;
            ROS_INFO("Full rotation completed. Stopping the car.");
        }
    }
    
    pub.publish(msg);
}

int main(int argc, char** argv)
{   
    label = 'b';

    ros::init(argc, argv, "rotation");
    ros::NodeHandle nh;

    
    
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    subscriber = nh.subscribe("objects",1000,callback);


    ros::Rate rate(10);
    
    while (ros::ok()) {
        if(!isStop){
            controlCar(cmd_vel_pub);
            ros::spinOnce();
            rate.sleep();   
        }else{
            break;
        }
        
    }
    if(isFound){
        return 1;
    }else{
        return 0;
    }
}