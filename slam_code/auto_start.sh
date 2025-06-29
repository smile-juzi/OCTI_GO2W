#!/bin/bash

sleep 10
# 切换到工作空间
cd /home/orangepi/slam_ws

# 加载 ROS 环境
source /opt/ros/noetic/setup.bash
source /home/orangepi/slam_ws/devel/setup.bash

# 启动雷达驱动节点
xfce4-terminal --title="Livox Driver" --hold -e "bash -c 'roslaunch livox_ros_driver2 msg_MID360.launch'" &

# 等待 5 秒
sleep 5

# 启动地图定位模块
xfce4-terminal --title="Map Switch" --hold -e "bash -c 'roslaunch map_switch project.launch'" &
