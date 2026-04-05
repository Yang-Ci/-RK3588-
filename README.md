# UCAR 3.0 - 基于 RK3588 的 ROS 自主智能车

一个基于 **ROS** 和 **Rockchip RK3588** 平台的自主智能车系统，专为全国大学生智能汽车竞赛设计。集成了 NPU 硬件加速的 YOLOv5 目标检测、SLAM 建图与导航、语音交互、交通灯识别、巡线避障等核心功能。

## 功能特性

- **目标检测**: 基于 YOLOv5 + RKNN NPU 加速，支持 9 类物品识别（水果、蔬菜、甜点）
- **自主导航**: AMCL 定位 + Move Base 全局/TEB 局部路径规划
- **SLAM 建图**: GMapping 实时建图
- **语音交互**: 科大讯飞 AIUI 语音识别与唤醒
- **交通灯识别**: HSV 颜色空间红绿灯检测
- **巡线避障**: PID 巡线控制 + 障碍物规避状态机
- **二维码识别**: 房间标识识别
- **任务管理**: 多阶段竞赛任务调度与状态管理

## 系统架构

```
┌──────────────────────────────────────────────────────────┐
│                    UCAR 3.0 系统                          │
└──────────────────────────────────────────────────────────┘
                          │
    ┌─────────────────────┼─────────────────────┐
    │                     │                     │
┌───▼───┐          ┌──────▼──────┐        ┌────▼────┐
│ 传感器 │          │  RK3588 计算 │        │ 执行器  │
│ USB相机│──图像───►│   ROS 核心   │──cmd_vel─►│底盘MCU │
│ YDLIDAR│──扫描───►│              │        │         │
│ IMU   │──数据───►│              │        │         │
└───────┘          └──────┬───────┘        └─────────┘
                          │
         ┌────────────────┼────────────────┐
         │                │                │
   ┌─────▼─────┐   ┌─────▼─────┐   ┌─────▼─────┐
   │  环境感知  │   │  自主导航  │   │  任务管理  │
   │ rknn_ros  │   │ amcl      │   │ task_mgr  │
   │ YOLOv5   │   │ move_base │   │ room_det  │
   │ 交通灯   │   │ map_server│   │ final_obj │
   │ 巡线     │   │ teb_local │   │ wakeup    │
   │ 二维码   │   │ costmap   │   │ voice_pb  │
   └───────────┘   └───────────┘   └───────────┘
```

## 目录结构

```
src/
├── rknn_ros/                 # YOLOv5 + RKNN NPU 推理节点
├── ucar_object_detection/    # 目标检测与任务执行
├── ucar_nav/                 # 导航启动文件与参数配置
├── ucar_map/                 # SLAM 建图与地图文件
├── navigation/               # ROS 导航栈 (move_base, amcl, teb等)
├── color/                    # 交通灯颜色检测
├── line/                     # PID 巡线与避障
├── speech_command/           # 科大讯飞 AIUI 语音命令
├── wakeup_handler/           # 唤醒词处理
├── voice_playback/           # 语音播放
├── ucar_controller/          # 底盘驱动
├── ucar_camera/              # 相机驱动与二维码检测
├── ucar_manager/             # 任务状态管理
├── ucar_room/                # 房间检测
├── ucar_final/               # 最终结算
├── ucar_traffic/             # 交通灯语音反馈
├── ydlidar/                  # YDLIDAR 激光雷达驱动
├── fdilink_ahrs/             # IMU 驱动
├── usb_cam/                  # USB 相机驱动
├── object_information_msgs/  # 目标信息消息定义
├── detection_msgs/           # 检测消息定义
├── rviz/                     # RViz 可视化配置
├── waterplus_map_tools/      # 航点导航
├── multimaster_fkie/         # 多主机 ROS 同步
├── yolo_tools/               # YOLO 工具脚本
├── demo.py                   # 单张图片推理演示
└── 111.py                    # 实时相机推理演示
```

## 硬件要求

- **主控**: Rockchip RK3588 SoC (6 TOPS NPU)
- **相机**: USB 摄像头 (640x480, 30fps)
- **激光雷达**: YDLIDAR 2D
- **IMU**: FDILink AHRS
- **底盘**: 自定义 MCU 底盘控制器

## 软件依赖

- **ROS** Noetic/Melodic
- **RKNN Toolkit 2** v1.5.0
- **OpenCV**
- **YOLOv5** (已导出为 .rknn 格式)
- **科大讯飞 AIUI SDK**
- **Python 3**: numpy, cv2, rknnlite
- **C++11/17**

## 编译与运行

### 编译

```bash
cd ~/ucar_ws
catkin_make
source devel/setup.bash
```

### 完整系统（竞赛模式）

```bash
roslaunch ucar_nav ucar_navigation.launch
```

启动内容：底盘驱动、激光雷达、AMCL 定位、Move Base 导航、地图服务、RViz、语音命令、视觉/任务系统、唤醒处理、ROSBridge、航点导航、交通灯检测。

### 仅导航

```bash
roslaunch ucar_nav nav.launch
```

### 仅 YOLO 检测

```bash
roslaunch rknn_ros yolov5.launch
```

### SLAM 建图

```bash
roslaunch ucar_map ucar_gmapping.launch
```

### 仅视觉系统

```bash
roslaunch ucar_camera start_vision.launch
```

### 独立 Python 演示（无需 ROS）

```bash
# 单张图片推理
python3 demo.py

# 实时相机推理
python3 111.py
```

## 竞赛任务流程

1. **任务分配**: 通过语音或系统指令接收类别（水果/蔬菜/甜点）
2. **目标检测**: 使用 YOLOv5 检测环境中的物品
3. **物品选取**: 拾取两个匹配类别的物品
4. **房间导航**: 导航至正确房间（A/B/C），通过二维码或视觉标识识别
5. **交通灯处理**: 根据红绿灯检测停止/通行
6. **巡线行驶**: 沿赛道巡线，自动规避障碍物
7. **最终结算**: 播报完成的类别与两个物品名称

## 检测类别

| 类别 | 物品 |
|------|------|
| 水果 (fruit) | 苹果、香蕉、西瓜 |
| 蔬菜 (vegetable) | 番茄、土豆、辣椒 |
| 甜点 (dessert) | 牛奶、蛋糕、可乐 |

## 关键话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/usb_cam/image_raw` | sensor_msgs/Image | 相机图像 |
| `/objects` | object_information_msgs/Object | 检测结果 |
| `/cmd_vel` | geometry_msgs/Twist | 速度指令 |
| `/scan` | sensor_msgs/LaserScan | 激光雷达数据 |
| `/task_status/*` | std_msgs/String | 任务状态 |
| `/wakeup_event` | std_msgs/Bool | 唤醒事件 |

## License

本项目仅供学习与竞赛使用。
