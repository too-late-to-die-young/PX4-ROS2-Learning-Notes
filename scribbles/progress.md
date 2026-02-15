# 1.30
有事

# 1.31
安装plotjoggler（我用过）

装docker，会概念和基本指令（笔记写了一半）

实现轨迹追踪（要求mavros，但不适配ros2，考虑改用px4_msgs?）


| 概念   | px4_msgs             | 老系统 (MAVROS)                          |
| :----- | :----------------------------------- | :--------------------------------------- |
| 消息类型 | TrajectorySetpoint                   | mavros_msgs/PositionTarget               |
| 位置   | position[0], [1], [2]                | position.x, y, z                         |
| 速度   | velocity[0], [1], [2]                | velocity.x, y, z                         |
| 掩码 (Bitmask) | 默认全部有效 (或填 NaN) | type_mask (需要手动计算位运算)           |
| 坐标系 | NED (北-东-地)                       | ENU (东-北-天)                           |



# 2.1

ubuntu自带的截屏太巨大了，装了flameshot

学习pid调参

录包（只写了命令行，没有实操

移植以上代码到docker里面并完善笔记

遇到了一个匪夷所思的问题，plotjuggler能订阅in，但显示不出in。


# 2.2

水了一天，一直在看那个心得。

尝试了录包。

看了一点docker视频课。

试了试qgc虚拟摇杆，还是没法替代遥控器调参。

# 2.3
看两个docker的课，把docker basic command写完。

草 太多了 根本学不完

明天再修修 basic command，全部改用表格好了

明天如果没有新活的话就去试试子网通信和尝试解决共享网络遇到的问题

# 2.4 
pid油管课，docker 补完 控制理论

看了一天自控原理和pid，燃尽了

# 2.5
docker还没补完（悲）

读minimum snap

原文读到轨迹生成部分，先去把飞哥的课再看一遍

# 2.6
 minimum snap读完了，写了笔记（主要是理解逻辑用）

# 2.7
出去玩

开始手搓minimum snap

# 2.8
补课凸优化概念

手搓一维minimum snap（仅演示轨迹，无仿真），对比了两个时间分配方案

# 2.9
手搓三维minimum snap 以及 飞行走廊约束

# 2.10
读完闭式求解的论文
rviz可视化飞行通道

# 2.11
继续rviz可视化
出去玩（

# 2.12
可视化完成
读论文*Polynomial Trajectory Planning for Aggressive
Quadrotor Flight in Dense Indoor Environments*

# 2.13
玩

# 2.14
玩

# 2.15
完成闭式求解
开始看2020论文