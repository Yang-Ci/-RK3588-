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
bool pass_step = false;
bool nofind_FLAG = false;
int result_t =  0;
int result_get = 0;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int main(int argc, char** argv){
    ros::init(argc, argv, "send_goals_b");
    MoveBaseClient ac("move_base", true);
    uint8_t goal_number_n =10;
    uint8_t goal_number = 10;


    while(!ac.waitForServer( ros::Duration( 5.0 ) )){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal[goal_number_n];
//B qu
    goal[0].target_pose.pose.position.x = 1.01603;
    goal[0].target_pose.pose.position.y =  1.18518;
    goal[0].target_pose.pose.orientation.z =  -0.0358342;  
    goal[0].target_pose.pose.orientation.w = 0.999358;
//b->a
    // 第一个待发送的 目标点 在 map 坐标系下的坐标位置
    goal[1].target_pose.pose.position.x = -0.035367;
    goal[1].target_pose.pose.position.y =  0.0228656;
    goal[1].target_pose.pose.orientation.z =  0.99878;  
    goal[1].target_pose.pose.orientation.w = 0.0493771;
//f
    goal[2].target_pose.pose.position.x = -2.65837;
    goal[2].target_pose.pose.position.y = -1.67105;
    goal[2].target_pose.pose.orientation.z =  -0.725838;  
    goal[2].target_pose.pose.orientation.w = 0.687866; 
//souxun 
    goal[3].target_pose.pose.position.x = -1.62696;
    goal[3].target_pose.pose.position.y = 0.939058;
    goal[3].target_pose.pose.orientation.z = 0.921235;  
    goal[3].target_pose.pose.orientation.w = 0.389007;

    goal[4].target_pose.pose.position.x = -1.68676;
    goal[4].target_pose.pose.position.y = 0.929316;
    goal[4].target_pose.pose.orientation.z = 0.609921;  
    goal[4].target_pose.pose.orientation.w = 0.792462;

    goal[5].target_pose.pose.position.x = -1.6855;
    goal[5].target_pose.pose.position.y = 1.00275;
    goal[5].target_pose.pose.orientation.z = -0.0773504;  
    goal[5].target_pose.pose.orientation.w = 0.997004;

    goal[6].target_pose.pose.position.x = -1.74149;
    goal[6].target_pose.pose.position.y = 0.200506;
    goal[6].target_pose.pose.orientation.z = -0.645108;  
    goal[6].target_pose.pose.orientation.w = 0.764092;
//sahngpo
    goal[7].target_pose.pose.position.x = -1.70906;
    goal[7].target_pose.pose.position.y = 0.133194;
    goal[7].target_pose.pose.orientation.z = -0.0285193;  
    goal[7].target_pose.pose.orientation.w = 0.999593;// 第四个待发送的 目标点 在 map 坐标系下的坐标位置
//a-c
    goal[8].target_pose.pose.position.x =-0.0494958;
    goal[8].target_pose.pose.position.y = -0.00355166;
    goal[8].target_pose.pose.orientation.z = -0.702038;  
    goal[8].target_pose.pose.orientation.w = 0.712139;

    // 第5个待发送的 目标点 在 map 坐标系下的坐标位置
    // goal[6].target_pose.pose.position.x =0.905526;
    // goal[6].target_pose.pose.position.y = -0.5561;
    // goal[6].target_pose.pose.orientation.z = -0.0149141;  
    // goal[6].target_pose.pose.orientation.w = 0.999889; 
//finish
    goal[9].target_pose.pose.position.x =1.91945;
    goal[9].target_pose.pose.position.y = -0.101771;
    goal[9].target_pose.pose.orientation.z = 0.699346;  
    goal[9].target_pose.pose.orientation.w = 0.714784; 
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
            if(!pass_step){
                    goal[goal_number_n -goal_number].target_pose.header.frame_id = "map";
                     goal[goal_number_n -goal_number].target_pose.header.stamp = ros::Time::now();
                     BVS_FLAG = true;
                     ac.sendGoal(goal[goal_number_n -goal_number]);
                     ROS_INFO("Send NO. %d Goal !!!", goal_number_n-goal_number );
                break;
            }else{
                break;
            }
            case 4:
            if(!pass_step){
goal[goal_number_n -goal_number].target_pose.header.frame_id = "map";
                     goal[goal_number_n -goal_number].target_pose.header.stamp = ros::Time::now();
                    //  BVS_FLAG = true;
                     ac.sendGoal(goal[goal_number_n -goal_number]);
                     ROS_INFO("Send NO. %d Goal !!!", goal_number_n-goal_number );
                break;
            }else{
                break;
            }
                     
            case 5:
            if(!pass_step){
goal[goal_number_n -goal_number].target_pose.header.frame_id = "map";
                     goal[goal_number_n -goal_number].target_pose.header.stamp = ros::Time::now();
                    //  BVS_FLAG = true;
                     ac.sendGoal(goal[goal_number_n -goal_number]);
                     ROS_INFO("Send NO. %d Goal !!!", goal_number_n-goal_number );
                break;
            }else{
                break;
            }
            case 6:
            if(!pass_step){
goal[goal_number_n -goal_number].target_pose.header.frame_id = "map";
                     goal[goal_number_n -goal_number].target_pose.header.stamp = ros::Time::now();
                    //  BVS_FLAG = true;
                     ac.sendGoal(goal[goal_number_n -goal_number]);
                     ROS_INFO("Send NO. %d Goal !!!", goal_number_n-goal_number );
                     nofind_FLAG = true;
                break;
            }else{
                break;
            }
            case 7:
                     goal[goal_number_n -goal_number].target_pose.header.frame_id = "map";
                     goal[goal_number_n -goal_number].target_pose.header.stamp = ros::Time::now();
                     ac.sendGoal(goal[goal_number_n -goal_number]);
                     ROS_INFO("Send NO. %d Goal !!!", goal_number_n-goal_number );
                break;
            case 8:
                     goal[goal_number_n -goal_number].target_pose.header.frame_id = "map";
                     goal[goal_number_n -goal_number].target_pose.header.stamp = ros::Time::now();
                     ac.sendGoal(goal[goal_number_n -goal_number]);
                     ROS_INFO("Send NO. %d Goal !!!", goal_number_n-goal_number );
                break;
            case 9:
                     goal[goal_number_n -goal_number].target_pose.header.frame_id = "map";
                     goal[goal_number_n -goal_number].target_pose.header.stamp = ros::Time::now();
                     ac.sendGoal(goal[goal_number_n -goal_number]);
                     FINISH_FLAG = true;
                     ROS_INFO("Send NO. %d Goal !!!", goal_number_n-goal_number );
                break;
            // case 7:
            //          goal[goal_number_n -goal_number].target_pose.header.frame_id = "map";
            //          goal[goal_number_n -goal_number].target_pose.header.stamp = ros::Time::now();
            //          FINISH_FLAG = true;
            //          ac.sendGoal(goal[goal_number_n -goal_number]);
            //          ROS_INFO("Send NO. %d Goal !!!", goal_number_n-goal_number );
            //     break;
            
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
            if(BVS_FLAG && result_get == 0){
                if(result_t == 256){
                    std::string command = "rosrun send_goals send_goals_getB";
                    result_get = std::system(command.c_str());
                }
                if(result_t == 512){
                    std::string command = "rosrun send_goals send_goals_getV";
                    result_get = std::system(command.c_str());
                }
                if(result_t == 768){
                    std::string command = "rosrun send_goals send_goals_getS";
                    result_get = std::system(command.c_str());
                }
                if(result_get == 256){
                    pass_step = true;
                    BVS_FLAG = false;
                }
                if(nofind_FLAG && result_get == 0){
                    if(result_t == 256){
                        std::string command = "aplay /home/iflytek/Music/b.wav";
                        int resultx = std::system(command.c_str());
                        nofind_FLAG = false;
                    }
                    if(result_t == 512){
                        std::string command = "aplay /home/iflytek/Music/v.wav";
                        int resultx = std::system(command.c_str());
                        nofind_FLAG = false;
                    }
                    if(result_t == 768){
                        std::string command = "aplay /home/iflytek/Music/s.wav";
                        int resultx = std::system(command.c_str());
                        nofind_FLAG = false;
                    }
                }
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
