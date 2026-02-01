# PID调参和PlotJuggler

## 0.准备
安装PlotJuggler，用于实时绘图和数据分析。
问gemini就行。

## 1.PlotJuggler

跑起01-Trajectory-Tracking里的圆轨迹仿真，然后在工作空间的目录下新开一个终端：
```bash
# 同时 source ROS2 系统环境和你自己的工作空间
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 在这个终端启动 PlotJuggler
ros2 run plotjuggler plotjuggler
```
在streaming中订阅:
```/px4_ros_com/vehicle_trajectory_setpoint```
```/px4_ros_com/vehicle_local_position```

使用XY-plot模式更直观（我的代码里直接把Z设成定值了，Z可以先不管），但新版PlotJuggler里这个模式不是很显眼：
先选vehicle_trajectory_setpoint里的position[0]，然后按住Ctrl键再选position[1]，这时候按住鼠标右键把它们拖到空白tab里就会跳出XY-plot选项。
vehicle_local_position同理，选x和y，拖到和上面同一个tab里，适当调高buffer time，就能看到两个完整的圆轨迹。

![alt text](</images/control_and_tuning_1.png>)
这是有前馈的效果，其实已经很圆了。
稍微改了下先前的代码，取消前馈，在这个基础上调PID。


## 2.PID调参

### QGC调参
Vehicle Setup -> Parameters，搜索并修改参数：
-  ```MPC_XY_P```：位置环比例增益
- ```MPC_XY_VEL_P_ACC```：速度环比例增益
- ```MPC_XY_VEL_I_ACC```：速度环积分增益
- ```MPC_XY_VEL_D_ACC```：速度环微分增益

新版QGC提供了专门的可视化调参界面，直接用这个更好：
**Vehicle Setup -> PID Tuning**
*QGC里面还有Attitude Controller和Rate Controller，但这两个在px4的默认飞机模型里应该已经调好了，先不管。*

### PID思路
先在gazebo给无人机加一个扰动。找到Apply Force/Torque插件，在entity tree选中飞机，填入施力大小和方向（y：1000N），按一下APPLY FORCE就松手（不要一直按，会被击飞），然后立刻在QGC里调参。

![alt text](/images/control_and_tuning_2.png)
先调 **Velocity（内环）**，再调 **Position（外环）**。

#### Velocity：
##### P：MPC_XY_VEL_P_ACC
P小：回复慢，哪怕没有扰动也会有明显误差，但相对平稳（但如果I太大的话，P太小也会引起震荡）。
P大：回复快，误差小。会因为回复太猛产生震荡，需要用D来平滑一下。

##### I：MPC_XY_VEL_I_ACC
用于处理累计误差，太大会严重震荡。
如果I比较大的话，P过大或过小都会有明显震荡，在中间某个位置震荡比较小。

##### D：MPC_XY_VEL_D_ACC
用来弥补P太大导致的震荡问题（我们想要P带来的回复速度，但不想要震荡）。
因为是对P的补偿，效果基本上就是和P反着来。


#### Position

##### MPC_XY_P
这个在Plot Juggler的XY图中看更直观一些。P小的话容易飞偏，大的话也是容易震荡。

## 3.录包
### ROS2
工作空间下：
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
开始录制（指定保存文件名，方便后续查找）
ros2 bag record -o circle_test_v1 /mavros/local_position/pose /trajectory_setpoint
```

### PX4 ULog
在PX4 终端（或通过 QGC 里的控制台）：
```bash
#开始录制
logger start

#结束录制
logger stop
```

存放在PX4目录下的```build/px4_sitl_default/tmp/rootfs/log/```

这个东西可以传到Flight Review。