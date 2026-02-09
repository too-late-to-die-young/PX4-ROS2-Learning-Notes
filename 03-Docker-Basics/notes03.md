# Docker
## 0.准备

我学习Docker参考的一些课程：

[40分钟的Docker实战攻略，一期视频精通Docker](https://www.bilibili.com/video/BV1THKyzBER6/?share_source=copy_web&vd_source=6a65513384955cc33f848d6a6894e1a1)
[一份基于这个教程的笔记]()
我的基本指令文档是基于这个教程写的。

[菜鸟教程](https://www.runoob.com/docker/docker-tutorial.html)

[【教程向】机器人专业Docker入门指南【鱼香ros】](https://www.bilibili.com/video/BV1Eu4y1D76k/?share_source=copy_web&vd_source=6a65513384955cc33f848d6a6894e1a1)

首先安装Docker（略）。
权限设置免sudo：
```bash
# 1. 创建 docker 组（通常已存在）
sudo groupadd docker

# 2. 将当前用户 ${USER} 加入组
sudo usermod -aG docker $USER

# 3. 激活更改（或者重启电脑）
newgrp docker
```

拉取```ros:jazzy-ros-base```：
```bash
docker pull ros:jazzy-ros-base
```
从官方库直接拉取，常因为网络问题拉不下来，解决办法：
```bash
#用代理拉取（目前可用的）
docker pull docker.m.daocloud.io/library/ros:jazzy-ros-base
#用代理拉的镜像名字有点长，把名字改短
docker tag docker.m.daocloud.io/library/ros:jazzy-ros-base ros:jazzy-ros-base
```
或者更彻底一点，直接修改配置文件添加镜像源，编辑```/etc/docker/daemon.json```（如果没有就创建一个）：
```json
{
  "registry-mirrors": [
    "https://mirror.baidubce.com",
    "https://docker.m.daocloud.io",
    "https://reg-mirror.qiniu.com"
  ]
}
```
```bash
sudo systemctl daemon-reload
sudo systemctl restart docker
```

## 1.基本指令

常用基本指令写在basic_commands.md里面了。


## 2.创建镜像
写一个`Dockerfile`（就叫这个，没有后缀）：
```dockerfile
# 1. 指定基础镜像
FROM px4io/px4-dev-ros2-foxy:latest

# 2. 设置环境变量 (避免交互式安装弹窗)
ENV DEBIAN_FRONTEND=noninteractive

# 3. 设置工作目录
WORKDIR /home/user/ros2_ws

# 4. 安装系统依赖 (常用库)
RUN apt-get update && apt-get install -y \
    ros-foxy-mavros \
    ros-foxy-mavros-extras \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# 5. 复制本地文件到镜像
COPY ./src ./src

# 6. 运行编译指令
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build"

# 7. 暴露端口 (如 MAVLink 通信)
EXPOSE 14540/udp

# 8. 容器启动时默认执行的命令
CMD ["/bin/bash"]
```
在Dockerfile所在目录下开一个终端：
```bash
docker build -t [镜像名] [路径（通常是.）]
```
后面还有上传到Docker Hub，我懒得写了。


## 3.Docer网络
前面拉取的是缩水版ROS。完整带图形化界面的```desktop```版太巨大了，拉不下来，而且在Docker里跑笨重的图形化界面不太合理。

与宿主机共享网络（有两个奇葩报错没有解决）：
![alt text](/images/docker_basics_1.png)

子网通信:

## 4.Docker Compose
（例如在工作空间最顶层）创建`docker-compose.yml`：
```yml
version: '3.8'

services:
  # 服务 1：PX4 仿真器
  px4_sitl:
    image: px4io/px4-dev-ros2-foxy
    container_name: sitl_gazebo
    network_mode: "host"       # 对应 --network host
    volumes:
      - ./PX4-Autopilot:/src/PX4-Autopilot  # 对应 -v
    environment:
      - PX4_SIM_MODEL=gz_x500  # 对应 -e
    stdin_open: true           # 对应 -i
    tty: true                  # 对应 -t

  # 服务 2：ROS 2 控制节点
  offboard_control:
    image: my_uav_controller:v1
    container_name: uav_logic
    network_mode: "host"
    depends_on:
      - px4_sitl               # 确保仿真器先启动
    command: ros2 run my_package control_node
```
在该目录下：
```bash
docker-compose up
```
其他详见basic-commands.md