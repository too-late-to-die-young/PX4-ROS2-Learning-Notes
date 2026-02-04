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


## 2.QGC调参
[QGC官方文档](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/tuning_px4.html)
主要介绍了如何使用QGC调参，没有经验技巧性的教程。

Vehicle Setup -> Parameters，搜索并修改参数：
-  ```MPC_XY_P```：位置环比例增益
- ```MPC_XY_VEL_P_ACC```：速度环比例增益
- ```MPC_XY_VEL_I_ACC```：速度环积分增益
- ```MPC_XY_VEL_D_ACC```：速度环微分增益

新版QGC提供了专门的可视化调参界面，直接用这个更好：
**Vehicle Setup -> PID Tuning**


可以先在gazebo给无人机加一个扰动。找到Apply Force/Torque插件，在entity tree选中飞机，填入施力大小和方向（y：1000N），按一下APPLY FORCE就松手（不要一直按，会被击飞）。
![alt text](/images/control_and_tuning_2.png)
更好的解决办法是使用游戏手柄或实体遥控器主动给阶跃信号，但我没有，只有QGC的虚拟摇杆。

先调 **Velocity（内环）**，再调 **Position（外环）**。
*QGC里面还有Attitude Controller和Rate Controller，但这两个在px4的默认飞机仿真模型里应该已经调好了，先不管。*



#### Velocity：

调参顺序：P I D（存疑）。
##### P：MPC_XY_VEL_P_ACC
P小：回复慢，哪怕没有扰动也会有明显误差，但相对平稳（但如果I太大的话，P太小也会引起振荡）。
P大：回复快，误差小。会因为回复太猛产生振荡，需要用D来平滑一下。
只有P的话可能会有稳态误差（因为趋近于期望位置时加速度不断减小，在某时刻P提供的推力被阻力抵消，很可能在到达期望位置前飞机已经停下来了）,所以还需要I。

##### I：MPC_XY_VEL_I_ACC
用于消除稳态误差，太大会严重振荡/超调。

##### D：MPC_XY_VEL_D_ACC
用来弥补PI太大导致的振荡问题（我们想要PI带来的回复速度，但不想要振荡）。
因为是对PI的补偿，效果基本上就是和P反着来。


#### Position

##### MPC_XY_P
这个在Plot Juggler的XY图中看更直观一些。P小的话容易飞偏，大的话也是容易震荡。

## 3.录包
### ROS2
ROS话题信息等。
工作空间下：
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
#开始录制（指定保存文件名和位置）
ros2 bag record -o ~/文档/PX4-ROS2-Learning-Notes/bags/circle_test_v1 /mavros/local_position/pose /fmu/in/trajectory_setpoint
```
```Ctrl + C```停止录包。

ROS Bag可以在Plotjuggler里面打开查看。

### PX4 ULog
飞控信息。
在PX4 终端（或通过 QGC 里的控制台）：
```bash
#开始录制
logger start

#结束录制
logger stop
```
仿真默开启认录包，自动存放在PX4目录下的```build/px4_sitl_default/tmp/rootfs/log/```

这个东西可以传到[Flight Review](https://logs.px4.io/)。
我随便传了一个试试：https://logs.px4.io/plot_app?log=d9e413bc-d2aa-4242-aac7-7986c2f4f7b1


## 4.PID理论补充

### 资料
对于无人机PID调参，绝大部分资料是关于BF/穿越机飞控的，但这些资料同样有参考价值。

#### 时域分析
##### 1.[【透彻理解穿越机PID原理 底层逻辑和调节方法】](https://www.bilibili.com/video/BV1ou4y1D7VW/?share_source=copy_web&vd_source=6a65513384955cc33f848d6a6894e1a1)
最浅显的一个原理讲解。
提到了很重要的一个点：结构和动力是PID参数设置的最主要影响因素，所以对于标准轴距的机架（如 450、250），飞控默认参数就能表现出很不错的性能（事先已经针对结构做过大量适配）。[视频里提到的文章](https://www.sciencedirect.com/science/article/pii/S2352146519301875)

#### 频域分析
##### 2.[【自动控制原理】12_PID控制器_Matlab/Simulink仿真](https://www.bilibili.com/video/BV1xQ4y1T7yv/?share_source=copy_web&vd_source=6a65513384955cc33f848d6a6894e1a1)
一套非常好的自控原理课程，拉普拉斯变换部分可以先看他的[前置课程](https://www.bilibili.com/video/BV1ib411S7At/?share_source=copy_web&vd_source=6a65513384955cc33f848d6a6894e1a1)（看4~6课）。

##### 3.[【自动控制原理】2_稳定性分析_极点_Stability](https://www.bilibili.com/video/BV1s4411X7qd/?share_source=copy_web&vd_source=6a65513384955cc33f848d6a6894e1a1)

##### 4.[【重新认识穿越机PID 频域分析原理和调机工具】](https://www.bilibili.com/video/BV12hEwzyEgS/?share_source=copy_web&vd_source=6a65513384955cc33f848d6a6894e1a1)

![alt text](/images/control_and_tuning_3.jpeg)
![alt text](/images/control_and_tuning_4.jpeg)
![alt text](/images/control_and_tuning_7.jpeg)
带宽并不会直接体现在QGC界面，但可以理解为外环的响应要比内环“肉”一些。

#### 实战
##### 5.[【PX4姿态环PID在线调参经验分享】](https://www.bilibili.com/video/BV1JK411p7H6/?share_source=copy_web&vd_source=6a65513384955cc33f848d6a6894e1a1)[【PX4位置环PID在线调参经验分享】](https://www.bilibili.com/video/BV1Xz411i7CB/?share_source=copy_web&vd_source=6a65513384955cc33f848d6a6894e1a1)

##### 6.[油管上的PX4调参实战](https://youtu.be/PQOXJpstZx4?si=jMC7fzAOD9JmBAcs)
非常细致，是上面教程的终极合订本，纯英文，推荐从p2或p3开始看。

从下往上调参：
![alt text](/images/control_and_tuning_5.png)

###### rate
row，pitch，yaw操作基本相同，注意yaw的d通常设为0（相当于只用调P和K），且yaw可使用前馈（MC_YAWRATE_FF）。悬停飞机之后分别沿row，pitch，yaw方向拨动摇杆，降落飞机并获取飞控日志。

对PD平衡：
Setpoint（设定值）的峰值显著高于 Estimated（估计值/实际值），说明系统处于过阻尼（Overdamped）状态。减小D的10%-20%。
也可以结合阶跃响应图判断（有个小尖尖or回复太慢）。

PD完成后，调节K值：
相当于一起调节PID，对PD结果影响不大，但能改善Estimated对于Setpoint的追踪响应速度。K过大会引起超调振荡。增加K至产生振荡，回调20%-30%。对于追求悬停稳定的无人机，K可以略低。

I：
一般调完PDK之后不用大动I。对于yaw来说，有时候需要增加I来达到Setpoint峰值。

###### angle
只有P一个可调

###### Altitude
设置最大和最小速度。悬停飞机之后做快速的爬升和下降。同样先做好PD，在一起增加PID，必要时降低P修复underunderdamped，

###### Velocity Horiz.
设置最大速度，悬停飞机之后做四五个水平来回。（后面和前面说的差不多，不写了）。