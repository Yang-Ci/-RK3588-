#include "ros/ros.h"
#include "/home/iflytek/ucar_ws/devel/include/object_information_msgs/Object.h"  // 替换为实际包名
#include <map>
#include <string>

ros::Subscriber subscriber;

ros::Timer timer;
std::string label;
std::map<std::string, int> label_count;
int count = 0;
bool FLAG_t = false;
void callback(const object_information_msgs::Object::ConstPtr& msg)
    {
        static ros::Time lastCalledTime = ros::Time::now();  
        lastCalledTime = ros::Time::now();

        if (count < 100)
        {
            std::string label = msg->label;
            if(label == "t1" || label == "t2" || label == "t3"){
                label_count[label]++;
                FLAG_t = true;
            }
            count++;
        }
        if (count == 100)
        {   timer.stop();
            std::string most_frequent_label;
            int max_count = 0;
            for(const auto& pair : label_count){
                if(pair.second > max_count){
                    max_count = pair.second;
                    most_frequent_label = pair.first;
                }
            }
            label = most_frequent_label.c_str();
            ROS_INFO("The most frequent label is:%s with count: %d",most_frequent_label.c_str(),max_count);
            subscriber.shutdown();
        }
    }


// 定时器回调函数，用于检查callback是否在指定时间内被调用  
void checkCallbackTimer(const ros::TimerEvent& e) {  
    static ros::Time lastCalledTime = ros::Time::now(); // 初始化时间戳  
    ros::Duration duration = ros::Time::now() - lastCalledTime;  
  
    // 假设我们希望在5秒内没有调用callback时触发某些操作  
    if (duration.toSec() > 8.0) {  
        ROS_INFO("Callback function has not been called for more than 5 seconds.");  
        subscriber.shutdown();
        count = 100;
        timer.stop();
        
    }  
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_subscriber_node");



    ros::NodeHandle nh;
    // 设置定时器，每秒检查一次
    timer = nh.createTimer(ros::Duration(1.0), checkCallbackTimer);
    subscriber = nh.subscribe("objects",1000,callback);
    while(ros::ok()){
        ros::spinOnce();
        if(count >= 100){break;}
    }
    if(label == "t1"){
        std::string command = "aplay /home/iflytek/Music/t1.wav";
            int result = std::system(command.c_str());
            if(result != -1){
             ROS_INFO("zheng que zhixing");
            }
    ROS_INFO("ss%s",label.c_str());
    return 1;
    }if(label == "t2"){
        std::string command = "aplay /home/iflytek/Music/t2.wav";
            int result = std::system(command.c_str());
            if(result != -1){
             ROS_INFO("zheng que zhixing");
            }
    ROS_INFO("ss%s",label.c_str());
    return 2;
    }if(label == "t3"){
        std::string command = "aplay /home/iflytek/Music/t3.wav";
            int result = std::system(command.c_str());
            if(result != -1){
             ROS_INFO("zheng que zhixing");
            }
    ROS_INFO("ss%s",label.c_str());
    return 3;
    }
    if(!FLAG_t){
        std::string command = "aplay /home/iflytek/Music/t1.wav";
            int result = std::system(command.c_str());
            if(result != -1){
             ROS_INFO("zheng que zhixing");
            }
    ROS_INFO("ss%s",label.c_str());
    return 1;
    }
    ROS_INFO("ss%s",label.c_str());
    
    return 0;
}
