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
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
ros::Subscriber subscriber;
ros::Timer timer;
int count = 0;
std::string label;
int num;
bool FLAG = false;
double x_map_target;
double y_map_target;
double z;
double w;
// 相机内参
double fx = 425.7669112642185; // 示例值
double fy = 427.2781006366426; // 示例值
double cx = 323.7936842081658; // 示例值
double cy = 241.0551555157927; // 示例值
// 相机在小车坐标系下的位置
double x_cam = 0.19; // 示例值
double y_cam = 0.0; // 示例值
double z_cam = 0.135; // 示例值

// 目标物体的实际宽度（单位：米）
double W_real = 0; // 示例值

// 目标框的像素宽度和高度
double W_pixel = 0; // 示例值
double H_pixel = 0; // 示例值

void callback(const object_information_msgs::Object::ConstPtr& msg)
    {
        static ros::Time lastCalledTime = ros::Time::now();  
        lastCalledTime = ros::Time::now();
        if (count < 100)
        {
            if(label == msg->label){
                W_pixel += msg->size.y; //di
                H_pixel += msg->size.x; //gao
                num++;
                FLAG = true;
                
            }
            count++;
        }
        if (count == 100)
        {   timer.stop();
            W_pixel = W_pixel/num;
            H_pixel = H_pixel/num;
            if((H_pixel - W_pixel) > 0 && H_pixel >= 1.5*W_pixel){
                W_real = 0.11;
            }
            if((H_pixel/W_pixel)>0.9 && (H_pixel/W_pixel)<0.99){
                W_real = 0.23;
            }
            subscriber.shutdown();
            timer.stop();
        }
    }

// 定时器回调函数，用于检查callback是否在指定时间内被调用  
void checkCallbackTimer(const ros::TimerEvent& e) {  
    static ros::Time lastCalledTime = ros::Time::now(); // 初始化时间戳  
    ros::Duration duration = ros::Time::now() - lastCalledTime;  
  
    // 假设我们希望在5秒内没有调用callback时触发某些操作  
    if (duration.toSec() > 3.0) {  
        ROS_INFO("Callback function has not been called for more than 5 seconds.");  
        subscriber.shutdown();
        count = 100;
        FLAG = false;
        timer.stop();
        
    }  
}  





int main(int argc, char** argv)
{   
        label = 'b';
        W_real = 0.3;


    ros::init(argc, argv, "object_publisher");
    ros::NodeHandle n;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10.0);
    subscriber = n.subscribe("objects",1000,callback);
    // 设置定时器，每秒检查一次
    timer = n.createTimer(ros::Duration(1.0), checkCallbackTimer);
    while(ros::ok()){
        ros::spinOnce();
        if(count >= 100){break;}
    }

    int overcount = 0;
    if(FLAG){
    while (n.ok()) {
        geometry_msgs::TransformStamped transformStamped;
        try {
            rate.sleep();
            transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0),ros::Duration(5.0));
        } catch (tf2::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        double x_car = transformStamped.transform.translation.x;
        double y_car = transformStamped.transform.translation.y;
        double z_car = transformStamped.transform.translation.z;
        // ROS_INFO("x %f  y  %f  z   %f",x_car,y_car,z_car);
        tf2::Quaternion q(
            transformStamped.transform.rotation.x,
            transformStamped.transform.rotation.y,
            transformStamped.transform.rotation.z,
            transformStamped.transform.rotation.w
        );
        tf2::Matrix3x3 m(q);
        z = transformStamped.transform.rotation.z;
        w = transformStamped.transform.rotation.w;
        // ROS_INFO("jiaodu %f",transformStamped.transform.rotation.z);
        // ROS_INFO("jiaodu %f",transformStamped.transform.rotation.w);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        double theta_car = yaw;

        // 估算目标物体的深度
        double d = W_real * fx / W_pixel - 0.3;
        ROS_INFO("deep   %f",d);

        // 转换到map坐标系
        x_map_target = x_car + d * cos((theta_car*60)* (M_PI / 180.0));
        y_map_target = y_car + d * sin((theta_car*60)* (M_PI / 180.0));

        ROS_INFO("x %f  y  %f    thefa  %f",x_map_target,y_map_target,theta_car);
        // kuan_pub.publish(obj_msg);

        rate.sleep();
        overcount++;
        if(overcount >= 10){break;}
    }

    MoveBaseClient ac("move_base", true);
    uint8_t goal_number = 1;
    while(!ac.waitForServer( ros::Duration( 5.0 ) )){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal[1];

    goal[0].target_pose.pose.position.x = x_map_target;
    goal[0].target_pose.pose.position.y =  y_map_target;
    goal[0].target_pose.pose.orientation.z =  z;  
    goal[0].target_pose.pose.orientation.w = w;

    ROS_INFO(" Init success!!! ");
    while(goal_number )    // total is 4 goals
    {
        switch( (1 - goal_number) ){
            case 0:
                     goal[1 -goal_number].target_pose.header.frame_id = "map";
                     goal[1 -goal_number].target_pose.header.stamp = ros::Time::now();
                     ac.sendGoal(goal[1 -goal_number]);
                     ROS_INFO("Send NO. %d Goal !!!",1-goal_number );
                break;
            default:
                break;
        }
        ros::Duration timeout(10.0); // 10 seconds

        // Wait for the result with timeout
        bool finished_before_timeout = ac.waitForResult(timeout);
        if (finished_before_timeout) {
            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("The NO. %d Goal achieved success !!!", 1 - goal_number);
                goal_number--;
            } else {
                ROS_WARN("The NO. %d Goal Planning Failed for some reason", 1 - goal_number);
                goal_number--;
            }
        }
        else {
                // Handle the timeout scenario
                ROS_WARN("The NO. %d Goal did not finish within the timeout period", 1 - goal_number);
                ac.cancelGoal(); // Optionally, cancel the goal if it didn't finish in time
                goal_number--;
        }
        

        // ac.waitForResult();
        // if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        //     ROS_INFO("The NO. %d Goal achieved success !!!", 1-goal_number );
        //     goal_number -- ;
        // }else{ROS_WARN("The NO. %d Goal Planning Failed for some reason",1-goal_number); 
        //     goal_number --;
        // }
    }
    ROS_INFO("i'm over");
    std::string command = "aplay /home/iflytek/Music/b.wav";
            int result = std::system(command.c_str());
    return 1;
    }else{
        return 0;
    }
}