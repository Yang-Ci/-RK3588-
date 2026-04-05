#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include<iostream>
using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int main(int argc, char** argv){
    ros::init(argc, argv, "send_goals_node");
    MoveBaseClient ac("move_base", true);
    uint8_t goal_number = 6;

    while(!ac.waitForServer( ros::Duration( 5.0 ) )){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal[6];

    goal[0].target_pose.pose.position.x = 1.01603;
    goal[0].target_pose.pose.position.y =  1.18518;
    goal[0].target_pose.pose.orientation.z =  -0.0358342;  
    goal[0].target_pose.pose.orientation.w = 0.999358;

    // 第一个待发送的 目标点 在 map 坐标系下的坐标位置
    goal[1].target_pose.pose.position.x = -0.035367;
    goal[1].target_pose.pose.position.y =  0.0228656;
    goal[1].target_pose.pose.orientation.z =  0.99878;  
    goal[1].target_pose.pose.orientation.w = 0.0493771;  

    // 第二个待发送的 目标点 在 map 坐标系下的坐标位置
    goal[2].target_pose.pose.position.x = -2.65837;
    goal[2].target_pose.pose.position.y = -1.59105;
    goal[2].target_pose.pose.orientation.z =  -0.725838;  
    goal[2].target_pose.pose.orientation.w = 0.687866;  

    // 第三个待发送的 目标点 在 map 坐标系下的坐标位置
    goal[3].target_pose.pose.position.x = -1.70906;
    goal[3].target_pose.pose.position.y = 0.133194;
    goal[3].target_pose.pose.orientation.z = -0.0285193;  
    goal[3].target_pose.pose.orientation.w = 0.999593;  

    // 第四个待发送的 目标点 在 map 坐标系下的坐标位置
    goal[4].target_pose.pose.position.x =-0.0494958;
    goal[4].target_pose.pose.position.y = -0.00355166;
    goal[4].target_pose.pose.orientation.z = -0.702038;  
    goal[4].target_pose.pose.orientation.w = 0.712139; 

    // 第5个待发送的 目标点 在 map 坐标系下的坐标位置
    goal[5].target_pose.pose.position.x =1.91945;
    goal[5].target_pose.pose.position.y = -0.101771;
    goal[5].target_pose.pose.orientation.z = 0.699346;  
    goal[5].target_pose.pose.orientation.w = 0.714784; 

    // // 第6个待发送的 目标点 在 map 坐标系下的坐标位置
    // goal[6].target_pose.pose.position.x =-1.53889;
    // goal[6].target_pose.pose.position.y = 1.48698;
    // goal[6].target_pose.pose.orientation.z = 0.682614;  
    // goal[6].target_pose.pose.orientation.w = 0.730779; 

    // // 第7个待发送的 目标点 在 map 坐标系下的坐标位置
    // goal[7].target_pose.pose.position.x =-1.02012;
    // goal[7].target_pose.pose.position.y = 0.834034;
    // goal[7].target_pose.pose.orientation.z = -0.00231833;  
    // goal[7].target_pose.pose.orientation.w = 0.999731; 

    // // 第8个待发送的 目标点 在 map 坐标系下的坐标位置
    // goal[8].target_pose.pose.position.x =-1.91751;
    // goal[8].target_pose.pose.position.y = 0.124109;
    // goal[8].target_pose.pose.orientation.z = 0.0176709;  
    // goal[8].target_pose.pose.orientation.w = 0.999844; 

    // // 第9个待发送的 目标点 在 map 坐标系下的坐标位置
    // goal[9].target_pose.pose.position.x = -0.0716921;
    // goal[9].target_pose.pose.position.y = 0.0502247;
    // goal[9].target_pose.pose.orientation.z = -0.683194;  
    // goal[9].target_pose.pose.orientation.w = 0.730237;  

    ROS_INFO(" Init success!!! ");
    while(goal_number )    // total is 4 goals
    {
        switch( (6 - goal_number) ){
            case 0:
                     goal[6 -goal_number].target_pose.header.frame_id = "map";
                     goal[6 -goal_number].target_pose.header.stamp = ros::Time::now();
                     ac.sendGoal(goal[6 -goal_number]);
                     ROS_INFO("Send NO. %d Goal !!!", 6-goal_number );
                break;
            case 1:
                     goal[6 -goal_number].target_pose.header.frame_id = "map";
                     goal[6 -goal_number].target_pose.header.stamp = ros::Time::now();
                     ac.sendGoal(goal[6 -goal_number]);
                     ROS_INFO("Send NO. %d Goal !!!", 6-goal_number );
                break;
            case 2:
                     goal[6 -goal_number].target_pose.header.frame_id = "map";
                     goal[6 -goal_number].target_pose.header.stamp = ros::Time::now();
                     ac.sendGoal(goal[6 -goal_number]);
                     ROS_INFO("Send NO. %d Goal !!!", 6-goal_number );
                break;
            case 3:
                     goal[6 -goal_number].target_pose.header.frame_id = "map";
                     goal[6 -goal_number].target_pose.header.stamp = ros::Time::now();
                     ac.sendGoal(goal[6 -goal_number]);
                     ROS_INFO("Send NO. %d Goal !!!", 6-goal_number );
                break;
            case 4:
                     goal[6 -goal_number].target_pose.header.frame_id = "map";
                     goal[6 -goal_number].target_pose.header.stamp = ros::Time::now();
                     ac.sendGoal(goal[6 -goal_number]);
                     ROS_INFO("Send NO. %d Goal !!!", 6-goal_number );
                break;
            case 5:
                     goal[6 -goal_number].target_pose.header.frame_id = "map";
                     goal[6 -goal_number].target_pose.header.stamp = ros::Time::now();
                     ac.sendGoal(goal[6 -goal_number]);
                     ROS_INFO("Send NO. %d Goal !!!", 6-goal_number );
                break;
            // case 6:
            //          goal[6 -goal_number].target_pose.header.frame_id = "map";
            //          goal[6 -goal_number].target_pose.header.stamp = ros::Time::now();
            //          ac.sendGoal(goal[6 -goal_number]);
            //          ROS_INFO("Send NO. %d Goal !!!", 10-goal_number );
            //     break;
            // case 7:
            //          goal[10 -goal_number].target_pose.header.frame_id = "map";
            //          goal[10 -goal_number].target_pose.header.stamp = ros::Time::now();
            //          ac.sendGoal(goal[10 -goal_number]);
            //          ROS_INFO("Send NO. %d Goal !!!", 10-goal_number );
            //     break;
            // case 8:
            //          goal[10 -goal_number].target_pose.header.frame_id = "map";
            //          goal[10 -goal_number].target_pose.header.stamp = ros::Time::now();
            //          ac.sendGoal(goal[10 -goal_number]);
            //          ROS_INFO("Send NO. %d Goal !!!", 10-goal_number );
            //     break;
            // case 9:
            //          goal[10 -goal_number].target_pose.header.frame_id = "map";
            //          goal[10 -goal_number].target_pose.header.stamp = ros::Time::now();
            //          ac.sendGoal(goal[10 -goal_number]);
            //          ROS_INFO("Send NO. %d Goal !!!", 10-goal_number );
            //     break;
            default:
                break;
        }
        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("The NO. %d Goal achieved success !!!", 6-goal_number );
            goal_number -- ;
        }else{ROS_WARN("The NO. %d Goal Planning Failed for some reason",6-goal_number); }
    }
  return 0;}
