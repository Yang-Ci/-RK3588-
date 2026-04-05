#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <string>

#include "/home/iflytek/ucar_ws/devel/include/object_information_msgs/Object.h"
using namespace std;

bool T_FLAG =  true;
bool F_FLAG = false;
bool BVS_FLAG = false;
bool FINISH_FLAG = false;
int result_t =  0;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int main(int argc, char** argv){
    ros::init(argc, argv, "send_goals_b");
    MoveBaseClient ac("move_base", true);
    uint8_t goal_number_n = 7;
    uint8_t goal_number = 7;


    while(!ac.waitForServer( ros::Duration( 5.0 ) )){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal[goal_number_n];

    goal[0].target_pose.pose.position.x = 1.01603;
    goal[0].target_pose.pose.position.y =  1.18518;
    goal[0].target_pose.pose.orientation.z =  -0.0358342;  
    goal[0].target_pose.pose.orientation.w = 0.999358;

    // 第一个待发送的 目标点 在 map 坐标系下的坐标位置
    goal[1].target_pose.pose.position.x = -0.035367;
    goal[1].target_pose.pose.position.y =  0.0228656;
    goal[1].target_pose.pose.orientation.z =  0.99878;  
    goal[1].target_pose.pose.orientation.w = 0.0493771;

    goal[2].target_pose.pose.position.x = -2.63837;
    goal[2].target_pose.pose.position.y = -1.71105;
    goal[2].target_pose.pose.orientation.z =  -0.725838;  
    goal[2].target_pose.pose.orientation.w = 0.687866; 

    goal[3].target_pose.pose.position.x = -1.65982;
    goal[3].target_pose.pose.position.y = 0.116697;
    goal[3].target_pose.pose.orientation.z = 0.694024;  
    goal[3].target_pose.pose.orientation.w = 0.719952;

    goal[4].target_pose.pose.position.x = -1.70906;
    goal[4].target_pose.pose.position.y = 0.133194;
    goal[4].target_pose.pose.orientation.z = -0.0285193;  
    goal[4].target_pose.pose.orientation.w = 0.999593;// 第四个待发送的 目标点 在 map 坐标系下的坐标位置

    goal[5].target_pose.pose.position.x =-0.0494958;
    goal[5].target_pose.pose.position.y = -0.00355166;
    goal[5].target_pose.pose.orientation.z = -0.702038;  
    goal[5].target_pose.pose.orientation.w = 0.712139;

    // 第5个待发送的 目标点 在 map 坐标系下的坐标位置
    goal[6].target_pose.pose.position.x =1.91945;
    goal[6].target_pose.pose.position.y = -0.111771;
    goal[6].target_pose.pose.orientation.z = 0.699346;  
    goal[6].target_pose.pose.orientation.w = 0.714784; 
    ROS_INFO(" Init success!!! ");
    while(goal_number )    // total is 4 goals
    {
        switch( (goal_number_n - goal_number) ){
            case 0:
                     goal[goal_number_n -goal_number].target_pose.header.frame_id = "map";
                     goal[goal_number_n -goal_number].target_pose.header.stamp = ros::Time::now();
                     ac.sendGoal(goal[goal_number_n -goal_number]);
                     ROS_INFO("Send NO. %d Goal !!!", goal_number_n-goal_number );
                break;
            case 1:
                     goal[goal_number_n -goal_number].target_pose.header.frame_id = "map";
                     goal[goal_number_n -goal_number].target_pose.header.stamp = ros::Time::now();
                     ac.sendGoal(goal[goal_number_n -goal_number]);
                     ROS_INFO("Send NO. %d Goal !!!", goal_number_n-goal_number );
                break;
            case 2:
                     goal[goal_number_n -goal_number].target_pose.header.frame_id = "map";
                     goal[goal_number_n -goal_number].target_pose.header.stamp = ros::Time::now();
                     F_FLAG = true;
                     ac.sendGoal(goal[goal_number_n -goal_number]);
                     ROS_INFO("Send NO. %d Goal !!!", goal_number_n-goal_number );
                break;
            case 3:
                     goal[goal_number_n -goal_number].target_pose.header.frame_id = "map";
                     goal[goal_number_n -goal_number].target_pose.header.stamp = ros::Time::now();
                     BVS_FLAG = true;
                     ac.sendGoal(goal[goal_number_n -goal_number]);
                     ROS_INFO("Send NO. %d Goal !!!", goal_number_n-goal_number );
                break;
            case 4:
                     goal[goal_number_n -goal_number].target_pose.header.frame_id = "map";
                     goal[goal_number_n -goal_number].target_pose.header.stamp = ros::Time::now();
                     ac.sendGoal(goal[goal_number_n -goal_number]);
                     ROS_INFO("Send NO. %d Goal !!!", goal_number_n-goal_number );
                break;
            case 5:
                     goal[goal_number_n -goal_number].target_pose.header.frame_id = "map";
                     goal[goal_number_n -goal_number].target_pose.header.stamp = ros::Time::now();
                     ac.sendGoal(goal[goal_number_n -goal_number]);
                     ROS_INFO("Send NO. %d Goal !!!", goal_number_n-goal_number );
                break;
            case 6:
                     goal[goal_number_n -goal_number].target_pose.header.frame_id = "map";
                     goal[goal_number_n -goal_number].target_pose.header.stamp = ros::Time::now();
                     FINISH_FLAG = true;
                     ac.sendGoal(goal[goal_number_n -goal_number]);
                     ROS_INFO("Send NO. %d Goal !!!", goal_number_n-goal_number );
                break;
            
            default:
                break;
        }








        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("The NO. %d Goal achieved success !!!", goal_number_n-goal_number );
            if(T_FLAG){
                std::string command = "rosrun send_goals send_goals_dingyue";
                result_t = std::system(command.c_str());
            }
            if(result_t != -1){
                T_FLAG = false;
            }
            if(F_FLAG){
                std::string command = "aplay /home/iflytek/Music/f.wav";
                int result_f = std::system(command.c_str());
                F_FLAG = false;
            }
            if(BVS_FLAG){
                if(result_t == 256){
                    std::string command = "aplay /home/iflytek/Music/b.wav";
                    int result_b = std::system(command.c_str());
                }
                if(result_t == 512){
                    std::string command = "aplay /home/iflytek/Music/v.wav";
                    int result_v = std::system(command.c_str());
                }
                if(result_t == 768){
                    std::string command = "aplay /home/iflytek/Music/s.wav";
                    int result_s = std::system(command.c_str());
                }
                BVS_FLAG = false;
            }
            if(FINISH_FLAG){
                std::string command = "aplay /home/iflytek/Music/finish.wav";
                int result_b = std::system(command.c_str());
                FINISH_FLAG = false;
            }

            goal_number -- ;
        }else{ROS_WARN("The NO. %d Goal Planning Failed for some reason",goal_number_n-goal_number); }
    }
    ros::spin();
  return 0;}
