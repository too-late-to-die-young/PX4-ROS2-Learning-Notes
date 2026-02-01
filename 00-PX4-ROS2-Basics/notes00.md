# 仿真

## 0. 准备工作
问gemini就行了

python环境问题，empy报错external…啥的（我也不知道这样解决有什么风险）：
```bash
pip install --user -U empy==3.3.4 pyros-genmsg setuptools --break-system-packages
```


## 1. 命令行启动仿真


在PX4-Autopilot目录下：
```bash
# 启动默认 x500 机型配合 Gazebo (新版)
make px4_sitl gz_x500
```
新终端，连接 Micro XRCE Agent：
```bash
# 如果是在本地运行
MicroXRCEAgent udp4 -p 8888
```
新终端，在工作空间目录下：
```bash
# 刷新当前工作空间的路径
source install/setup.bash

# 运行px4_ros_com自带的悬停测试节点
ros2 run px4_ros_com offboard_control
```

## 2. 截图


ROS2 PX4 QGC配置完成
![仿真1](/images/px4_ros2_1.png)

ROS2运行offboard_control节点悬停
![仿真2](/images/px4_ros2_2.png)