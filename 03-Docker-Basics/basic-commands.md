# Docker基本指令

参考：[40分钟的Docker实战攻略，一期视频精通Docker](https://www.bilibili.com/video/BV1THKyzBER6/?share_source=copy_web&vd_source=6a65513384955cc33f848d6a6894e1a1)

## 1. 基本指令

| 指令         | 作用                 | 备注 |
| :---------------| :---------------- | :-------------- |
| `docker pull [域名/仓库地址]/[作者名/组织名]/[镜像名]:[版本号/标签]` | 下载镜像             |官方库可以只写镜像名（没写版本号默认装最新）|
`docker pull --platform [架构类型] [镜像名]:[标签]`|下载镜像（对特殊CPU架构）|迷你主机等装镜像（一般情况下电脑会自行选择对应架构的镜像）|
| `docker images` |列出本地镜像|
|`docker rmi [ID]`|删除本地镜像|ID一般输入前几位就够了|
| `docker run [ID]`  |（自动拉取镜像）创建并运行容器|`run`的其他参数详见后|
| `docker ps`   |查看**运行中**的容器|`docker ps -a`查看所有容器（包括未运行）|
| `docker rm [ID]`   |删除容器|`docker rm -f [ID]`强制删除运行中的容器|
|`docker start [ID]`|启动容器||
|`docker stop [ID]`|停止容器||
|`docker inspect [ID]`|查看容器信息|
|`docker create`|创建容器|类似`run`但不立即启动|
|`docker logs [ID]`|查看日志|`docker logs -f [ID]`实时滚动查看日志|
|`docker exec [ID] ps -ef`|查看容器内正在运行的进程||
|`docker exec -it [ID] /bin/bash`|进入容器的交互式终端||
|`docker network create [网络名称]`|创建一个自定义桥接网络|只有自定义网络才支持容器名互访（DNS 解析）|



## 2.`docker run`详细参数
|指令|作用|备注|
|:-|:-|:-|
|`docker run -d`|后台运行|不占用当前终端|
|`docker run -p [宿主机端口]:[容器端口]`|端口映射||
|`docker run -v [宿主机目录]:[容器内目录]`|绑定挂载|用于需要长久保存的文件|
|`docker volume create [卷名]`<br>`docker run -v [卷名]:[容器内路径]`|命名卷挂载||
|`docker volume inspect [卷名]`|查看卷详情|查看卷在宿主机上的物理存储路径|
|`docker volume ls`|列出所有卷|查看当前系统中存在的所有 Docker 卷|
|`docker volume rm [卷名]`|删除卷|正在被容器使用的卷无法删除|
|`docker volume prune -a`|删除所有没有任何容器在使用的卷|`docker volume prune`删除匿名未使用卷|
|`docker run -e [变量名]=[变量值] [镜像名]`|设置环境变量|
|`docker run --name [自定义名称] [镜像名]`|命名容器|名字等效于ID且利于记忆|
|`-it --rm`|交互模式&运行后删除|临时调试一个容器|
|`--restart always`|自动重启|无论容器因为什么停止，立即尝试重启该容器|
|`--restart unless-stopped`|自动重启|除了手动停止的情况，自动重启该容器|
|`--network host`/`--net=host`|共享宿主机网络|机器人中最常用|
|`--network [网络名称]`|加入网络|同一子网中的两个容器可以通过名字相互访问|

## 3.Dockerfile格式
```dockerfile
# 指定基础镜像
FROM px4io/px4-dev-ros2-jazzy:latest

# 设置环境变量 (避免交互式安装弹窗)
ENV DEBIAN_FRONTEND=noninteractive

# 设置工作目录(有点像cd)
WORKDIR /home/user/ros2_ws  

# 复制本地文件到镜像
COPY ./src ./src

# 在镜像中安装系统依赖 (常用库)
RUN apt-get update && apt-get install -y \
    ros-foxy-mavros \
    ros-foxy-mavros-extras \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# 运行编译指令
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build"

# 说明提供服务的端口
EXPOSE 14540/udp

# 容器启动时默认执行的命令，ENTRYPOINT与之相似但优先级更高，不容易被覆盖
CMD ["/bin/bash"]
```
## 4.docker-compose格式

让ai写


