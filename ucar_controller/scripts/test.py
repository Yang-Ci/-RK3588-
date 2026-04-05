#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
 
# def turn_in_place(degrees, speed):
#     # 初始化ROS节点
#     rospy.init_node('turn_in_place', anonymous=True)
    
#     # 创建一个Publisher发布到/cmd_vel话题
#     vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
#     # 等待订阅的topic就绪
#     rospy.sleep(1)
    
#     # 计算旋转所需的时间
#     r = (degrees / 360) * 10.0
#     t = r / speed
    
#     # 创建Twist消息
#     twist = Twist()
#     twist.linear.x = 0
#     twist.linear.y = 0
#     twist.linear.z = 0
#     twist.angular.x = 0
#     twist.angular.y = 0
#     twist.angular.z = speed  # 角速度设定为speed弧度/秒
    
#     # 发送指令
#     vel_pub.publish(twist)
#     rospy.loginfo("Turning in place for %f seconds at speed %f rad/s", t, speed)
#     rospy.sleep(t)
    
#     # 停止移动
#     twist.angular.z = 0
#     vel_pub.publish(twist)
#     rospy.loginfo("Turn completed.")
#     rospy.sleep(1)
#     rospy.signal_shutdown("Turn completed.")
 
if __name__ == '__main__':
    # 小车原地掉头，90度，转速speed弧度/秒
    rospy.init_node('turn_in_place', anonymous=True)
    speed=1.5
    # 创建一个Publisher发布到/cmd_vel话题
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)
    # 等待订阅的topic就绪
    rospy.sleep(1)
    
    # 计算旋转所需的时间
    r = (90 / 360) * 10.0
    t = r / speed
    t1=rospy.Time.now().to_sec()
    
    while ((rospy.Time.now().to_sec()-t1)<(t+0.55)):
     
     twist = Twist()
     twist.linear.x = 0
     twist.linear.y = 0
     twist.linear.z = 0
     twist.angular.x = 0
     twist.angular.y = 0
     twist.angular.z = speed  # 角速度设定为speed弧度/秒
     vel_pub.publish(twist)

    
    # 停止移动
    twist.angular.z = 0
    vel_pub.publish(twist)
    rospy.signal_shutdown("Turn completed.")
    #turn_in_place(degrees=90, speed=speed)
