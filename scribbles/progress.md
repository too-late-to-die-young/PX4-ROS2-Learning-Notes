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

ubuntu自带的截屏太巨大了，push不上去，装了flameshot



学习pid调参
实物实验录包（？之后再学吧

移植以上代码到docker里面并完善笔记