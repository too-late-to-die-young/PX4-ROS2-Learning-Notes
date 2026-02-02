# Docker
## 0.准备
安装Docker（略）。

权限设置免sudo
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
#直接拉取，网络问题拉不下来

#用代理拉取

```


## 1.实现

前面拉取的是缩水版ROS。完整带图形化界面的```desktop```版太巨大了，拉不下来，而且在Docker里跑笨重的图形化界面不太合理。

![alt text](image.png)

