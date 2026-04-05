import cv2            # opencv 库函数头文件引用
import numpy as np    # python 库引用
import math           # python 库引用
import roslib
import sys
import rospy          # ros python 库引用
from cv_bridge import CvBridge ,CvBridgeError   # opencv 和 ROS 交互头文件
from geometry_msgs.msg import Twist             # 速度Topic 头文件
from sensor_msgs.msg import CompressedImage     # 压缩图像 头文件

# 指定该ROS节点程序的名字
rospy.init_node('linetrack',anonymous=True)
 
 # 指定将发布的速度话题名字及类型
cmd_vel_pub = rospy.Publisher('ucar/cmd_vel',Twist,queue_size = 10)

 #指定将发布的压缩图像话题名字及类型
image_pub = rospy.Publisher("line_detect_image/compressed",CompressedImage,queue_size=1)
cvBridge = CvBridge()
# 利用opencv 库函数，从摄像头采集一帧图像
cap = cv2.VideoCapture(0)
ret, cv_image = cap.read()

# 对图像进行裁剪
height, width, channels = cv_image.shape
descentre = 50
rows_to_watch = 100

crop_img = cv_image[int((height)/4 + descentre):int((height)/4 + (descentre+rows_to_watch))][1:width]

#将图像从RGB转换成HSV色彩空间
hsv = cv2.cvtColor(crop_img,cv2.COLOR_RGB2HSV)
# 设置需要提取的颜色的是绿色，然后确认其HSV空间下的范围
lower_green = np.array([35,43,46])
upper_green = np.array([77,255,255])

#制作模板，绿色部分和其他颜色二值化后变为白色和黑色
mask = cv2.inRange(hsv,lower_green ,upper_green)

#将模板图像和原始图像按照 像素 位相与，提取出目标颜色
res = cv2.bitwise_and(crop_img,crop_img,mask = mask)

m = cv2.moments(mask,False)
try:
     cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
except ZeroDivisionError:
     cx, cy = height/2, width/2
height, width, channels = cv_image.shape
error_x = cx - width / 2
twist_object = Twist()
# 设置固定的线速度值，单位 m/s
twist_object.linear.x = 0.05

# 将横向像素的偏差除以一个系数，该系数决定角速度值的大小，需要根据线速度和图像总宽度进行调节，才能达到比较好的循迹效果
twist_object.angular.z = error_x / 80

# 二值化图像的像素统计，即统计目标颜色区域所占的像素个数
no_red = cv2.countNonZero(mask)

# 添加角速度和目标颜色像素判别，避免过大的角速度出现，防止车疯狂旋转
if ((math.fabs(twist_object.angular.z)) < 10 and (no_red > 20000)):
   cmd_vel_pub.publish(twist_object)
   rospy.loginfo("ANGULAR VALUE SENT ===>"+str(twist_object.angular.z))
else:
   twist_object.linear.x = 0
   rospy.loginfo("Out of range Stop!!! " + str(twist_object.angular.z))
   twist_object.angular.z = 0
   cmd_vel_pub.publish(twist_object)
try:
  msg = CompressedImage()
  msg.header.stamp = rospy.Time.now()
  msg.format = "jpeg"
  # 这里发布的是前面制作的模板，即二值化图像
  msg.data = np.array(cv2.imencode('.jpg', mask)[1]). tobytes()
  image_pub.publish(msg)
except CvBridgeError as e:
  print(e)