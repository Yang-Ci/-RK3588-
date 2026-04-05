#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <cstdlib>  // system()

void wakeupCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data)
    {
        ROS_INFO("收到唤醒消息，启动导航launch文件");

        // 启动launch命令，后台执行
        int ret = system("bash -c 'source ~/ucar_ws/devel/setup.bash && rosrun ucar_nav se_nav'");
        if (ret == -1)
        {
            ROS_ERROR("启动导航失败");
        }
        else
        {
            ROS_INFO("导航启动成功");
        }
    }
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL, ""); // 支持中文输出 
    ros::init(argc, argv, "wakeup_listener");
    ros::NodeHandle nh;
    //wakeup_pub = nh.advertise<std_msgs::Bool>("/wakeup_event", 1);
    ros::Subscriber sub = nh.subscribe("/wakeup_event", 10, wakeupCallback);

    ROS_INFO("wakeup_listener 节点启动，等待唤醒消息...");

    ros::spin();

    return 0;
}
