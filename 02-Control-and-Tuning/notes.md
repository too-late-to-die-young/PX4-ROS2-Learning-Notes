# PID调参和PlotJuggler

## 0.准备
安装PlotJuggler，用于实时绘图和数据分析。
问gemini就行。

## 1.PlotJuggler

跑起仿真之后在工作空间的目录下新开一个终端：
```bash
# 同时 source ROS2 系统环境和你自己的工作空间
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 在这个终端启动 PlotJuggler
ros2 run plotjuggler plotjuggler
```
如果仿真在运行的话是可以从PlotJuggler中订阅话题数据的。

## 2.使用QGC调参
