# 智慧社区系统


## 1. 节点启动顺序

本系统运行需要依次启动以下节点：

```
终端1
启动智慧社区仿真环境

↓

终端2
启动YOLO视觉识别

↓

终端3
启动车牌识别

↓

终端4
启动红绿灯检测

↓

终端5
启动自主巡航导航
```

注意：

每打开一个新的终端，都需要重新加载ROS2环境：

```bash
source /opt/ros/humble/setup.bash
source ~/smart_community_ws/install/setup.bash
```


---

## 2. 编译与运行命令


### 终端1：启动智慧社区仿真环境

功能：

启动Gazebo智慧社区比赛场景，同时加载TurtleBot3 Burger机器人、激光雷达以及摄像头。


执行命令：

```bash
cd ~/smart_community_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

export TURTLEBOT3_MODEL=burger

ros2 launch smart_community full_system.launch.py
```


---

### 终端2：启动YOLO视觉识别

功能：

启动社区人员目标检测模块，实现目标识别与结果输出。


执行命令：

```bash
cd ~/smart_community_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

python3 -u ~/smart_community_ws/yolo_camera_test.py
```


---

### 终端3：启动车牌识别

功能：

启动车辆检测与车牌识别模块。


执行命令：

```bash
cd ~/smart_community_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

python3 -u ~/smart_community_ws/plate_recognition.py
```


---

### 终端4：启动红绿灯检测

功能：

启动交通信号灯状态检测模块。


执行命令：

```bash
cd ~/smart_community_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run smart_community traffic_light_detection
```


---

### 终端5：启动自主巡航导航

功能：

启动机器人自主巡航任务，实现多目标点导航。


执行命令：

```bash
cd ~/smart_community_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run smart_community patrol_navigation --ros-args \
-p use_sim_time:=true \
-p start_index:=1
```


---

## 3. 编译命令

首次运行或修改代码后，需要重新编译：

```bash
cd ~/smart_community_ws

source /opt/ros/humble/setup.bash

colcon build

source install/setup.bash
```


---

## 4. 系统运行说明

运行顺序：

1. 首先启动终端1，加载智慧社区Gazebo仿真环境。
2. 等待机器人模型生成完成。
3. 启动终端2，运行YOLO视觉识别。
4. 启动终端3，运行车牌识别。
5. 启动终端4，运行红绿灯检测。
6. 启动终端5，执行自主巡航导航任务。


系统整体流程：

```
Gazebo仿真环境

        ↓

TurtleBot3 Burger机器人

        ↓

视觉感知模块

        ↓

目标识别模块

        ↓

自主导航模块

        ↓

完成智慧社区任务
```