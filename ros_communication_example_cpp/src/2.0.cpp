// robot_a_client.cpp
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "robot_a_client");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("/robot_b/start_simulation");

      // 等待服务可用，设置超时为5秒
    if (!client.waitForExistence(ros::Duration(5.0))) {
        ROS_ERROR("在5秒超时时间内未发现服务 /robot_b/start_simulation");
        return 1;
    }




    // ...到达指定地点逻辑...
    ROS_INFO("A机器人等待到达目标点...");

        ros::Rate rate(10);
    while (ros::ok() && !arrived)
    {
        ros::spinOnce();
        rate.sleep();
    }

    
    ROS_INFO("A机器人已到达目标点，通知B机器人开始仿真任务...");

    std_srvs::Trigger srv;
    if (client.call(srv))
    {
        if (srv.response.success)
        {
            ROS_INFO("收到B机器人反馈：%s", srv.response.message.c_str());
            // 后续任务...
        }
        else
        {
            ROS_ERROR("B机器人仿真失败：%s", srv.response.message.c_str());
        }
    }
    else
    {
        ROS_ERROR("无法调用服务 /robot_b/start_simulation");
    }

    return 0;
}