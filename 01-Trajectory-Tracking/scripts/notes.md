# 轨迹跟踪

## 0. 一个问题
ROS1中，使用mavros的setpoint_raw接口实现轨迹跟踪，在ROS2jazzy中，使用px4_ros_com的TrajectorySetpoint消息实现轨迹跟踪。
mavros不适配ros2，考虑改用px4_msgs。

| 概念   | px4_msgs             | 老系统 (MAVROS)                          |
| :----- | :----------------------------------- | :--------------------------------------- |
| 消息类型 | TrajectorySetpoint                   | mavros_msgs/PositionTarget               |
| 位置   | position[0], [1], [2]                | position.x, y, z                         |
| 速度   | velocity[0], [1], [2]                | velocity.x, y, z                         |
| 掩码 (Bitmask) | 默认全部有效 (或填 NaN) | type_mask (需要手动计算位运算)           |
| 坐标系 | NED (北-东-地)                       | ENU (东-北-天)                           |


| 控制目标       | px4_msgs 设置                  | MAVROS type_mask (十进制/十六进制) |
|----------------|--------------------------------|------------------------------------|
| 仅位置         | pos=true, vel=false, acc=false | 3576 (0x0DF8)                      |
| 仅速度         | pos=false, vel=true, acc=false | 3527 (0x0DC7)                      |
| 全状态 | pos=true, vel=true, acc=true   | 3072 (0x0C00)                      |


## 1. 代码

基于px4_ros_com的offboard_control节点，修改为轨迹跟踪版本，令无人机绕圈飞行，完整代码见scripts/trajectory_tracking.cpp。

```cpp
#include <cmath> //引入数学库
```

```cpp
/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 *修改点：激活了速度和加速度开关，实现setpoint_raw调参逻辑
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = true;    //速度控制
	msg.acceleration = true;   //加速度控制
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 *修改点：示例代码为悬停，我将它改为圆形轨迹
 */
void OffboardControl::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};  //对应

	const double radius = 5.0;  //半径5m
	const double omega = 0.5;	//角速度rad/s
	const double altitude = -5.0;	//高度5m，新版使用NED系(北-东-地)
	
	static rclcpp::Time start_time = this->get_clock()->now();
	double t = (this->get_clock()->now() - start_time).seconds();

	// 1. 位置目标
	msg.position[0] = radius * std::cos(omega * t);	//x
	msg.position[1] = radius * std::sin(omega * t);	//y
	msg.position[2] = altitude;	//z

	// 2. 速度前馈 (一阶导数)
	msg.velocity[0] = -radius * omega * std::sin(omega * t);
	msg.velocity[1] =  radius * omega * std::cos(omega * t);
	msg.velocity[2] = 0.0;

	// 3. 加速度前馈 (二阶导数)
	msg.acceleration[0] = -radius * omega * omega * std::cos(omega * t);
	msg.acceleration[1] = -radius * omega * omega * std::sin(omega * t);
	msg.acceleration[2] = 0.0;

	msg.yaw = std::atan2(msg.velocity[1], msg.velocity[0]);
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);

}
```

## 2. 截图
![trajectory_tracking](/images/trajectory_tracking.png)
