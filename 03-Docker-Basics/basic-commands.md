# docker基本指令

## 1. 基本指令

| 指令         | 作用                 | 备注 |
| :----------- | :------------------- | :------------------- |
| `docker pull` | 下载镜像             |
| `docker run`  | 运行容器             |
| `docker ps`   | 查看运行中的容器     |
| `docker stop` | 停止容器             |
| `docker rm`   | 删除容器             |
| `docker images` | 列出本地镜像         |
| `docker tag` | 重命名镜像 |

草稿：

docker run -v	挂载卷 (Volume)	让容器能读写我 Ubuntu 里的笔记文件，数据同步。
docker run --rm	运行完后自动删除	临时测试用，不留垃圾，保持系统干净。

docker run -it --name my_first_ros_container ros:jazzy-ros-base /bin/bash

-it: 交互式模式，让你能看到命令行并输入。

--name: 给你的容器起个名字，不然系统会随机分配一个奇怪的名字（如 modest_curie）。




xhost +local:docker  “允许本地 Docker 容器访问我的屏幕”。每次重启电脑后，如果你要跑带界面的 Docker，都需要运行一次。

## 2. troubleshooting
### 权限设置免sudo

```bash
# 1. 创建 docker 组（通常已存在）
sudo groupadd docker

# 2. 将当前用户 ${USER} 加入组
sudo usermod -aG docker $USER

# 3. 激活更改（或者重启电脑）
newgrp docker
```

### 无法连接Docker Hub 官方服务器

以拉取ros-base为例，`docker pull ros:jazzy-ros-base`无法连接服务器。

```bash
# 1. 目前可用代理
docker pull docker.m.daocloud.io/library/ros:jazzy-ros-base

#2. 把名字改短
docker tag docker.m.daocloud.io/library/ros:jazzy-ros-base ros:jazzy-ros-base
```

### jazzy-desktop过于笨重无法拉取

在docker中跑代码，通过共享局域网在本机上运行图形化软件

### 容器配置无法修改
打包一个新的镜像