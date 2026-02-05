# Minimum Snap
原文：*Minimum snap trajectory generation and control for quadrotors*

## 1.微分平坦性
### 微分平坦是什么
可以找到一组极少数的核心变量（称为“平坦输出”，Flat Outputs），使得系统中所有的状态变量（位置、姿态、速度等）和所有的控制输入（电机的力矩、电压等），都可以由这几个核心变量及其各阶导数直接计算出来，而不需要解微分方程。

只要规划好了空间中一条顺滑的轨迹 σ（包含位置 x,y,z 和偏航角 ψ），那么无人机在任何时刻“应该处于什么姿态”以及“电机应该出多少力”，都是唯一确定的。

### 微分平坦的作用
无人机本身是一个高维（12维）的系统，如果一条轨迹里面要有12个变量太复杂了。这12个维度之间一定存在约束，微分平坦本质上就是对无人机系统进行了降维，使轨迹可以简单地表达出来。

### 证明逻辑

#### 1.位置 (x,y,z) 的二阶导数 → 总推力方向（决定了飞机的 Roll 和 Pitch）。

$$ \boldsymbol{t} = [\ddot{\sigma}_1, \ddot{\sigma}_2, \ddot{\sigma}_3 + g]^T $$

- $\ddot{\sigma}_1, \ddot{\sigma}_2, \ddot{\sigma}_3$：惯性系下的三维加速度 $(\ddot{r}_x, \ddot{r}_y, \ddot{r}_z)$。
- $g$：重力加速度。
- $\boldsymbol{t}$ 的本质：无人机为完成规划动作，电机需提供的**合力向量**（需克服重力、提供加速度）。

合力方向的单位向量即为机体 $\boldsymbol{z}_B$ 轴：
$$ \boldsymbol{z}_B = \frac{\boldsymbol{t}}{\|\boldsymbol{t}\|} $$

引入yaw（偏航角）：
$$ \sigma_4 = \psi $$

$$ \boldsymbol{x}_C = [\cos\sigma_4, \sin\sigma_4, 0]^T $$

该向量表示**世界坐标系（水平面）中，理想的机头指向**（还不是真正的机体X轴），其中 $\sigma_4$ 对应偏航角（Yaw）$\psi$。

现在求机体Y轴 $\boldsymbol{y}_B$：
$$ \boldsymbol{y}_B = \frac{\boldsymbol{z}_B \times \boldsymbol{x}_C}{\|\boldsymbol{z}_B \times \boldsymbol{x}_C\|} $$

确保飞机Y轴既垂直于推力方向，又与偏航角 $\psi$ 保持几何约束关系。


基于已确定的Y轴、Z轴，再次通过叉乘得到飞机**真正的**机头方向（机体X轴）：
$$ \boldsymbol{x}_B = \boldsymbol{y}_B \times \boldsymbol{z}_B $$

**作用**：飞行中飞机倾斜，真实机头方向 $\boldsymbol{x}_B$ 与水平理想方向 $\boldsymbol{x}_C$ 未必一致，但 $\boldsymbol{x}_B$ 由 $\boldsymbol{x}_C$ 投影修正得到，保证姿态唯一性。

至此得到机体坐标系（要求机头不是垂直向上/向下的）：

$$ \boldsymbol{x}_C \times \boldsymbol{z}_B \neq \boldsymbol{0} $$

$$ {}^W_B \boldsymbol{R} = \begin{bmatrix} \boldsymbol{x}_B & \boldsymbol{y}_B & \boldsymbol{z}_B \end{bmatrix} $$

#### 2.位置 的三阶导数 (Jerk) → 总推力方向的变化率（决定了飞机的角速度）。
前文由牛顿定律得到了：

$$ m\boldsymbol{a} = u_1 \boldsymbol{z}_B - mg \boldsymbol{z}_W $$

对上述方程两边关于时间求导，得到：

$$ m\dot{\boldsymbol{a}} = \dot{u}_1 \boldsymbol{z}_B + \boldsymbol{\omega} \times (u_1 \boldsymbol{z}_B) \tag{7} $$

$m\dot{\boldsymbol{a}}$ 为**质量与急动度（Jerk，$\dot{\boldsymbol{a}}$）的乘积**。急动度是加速度的时间变化率。

接下来，消除 $\dot{u}_1$（推力大小的变化率），仅保留机体角速度相关项。将 $\boldsymbol{\omega} \times \boldsymbol{z}_B$ 单独提取并定义为 $\boldsymbol{h}_\omega$，用这个新变量消去$\dot{u}_1$得到：
$$ \boldsymbol{h}_\omega = \frac{u_1}{m} \left( \dot{\boldsymbol{a}} - (\boldsymbol{z}_B \cdot \dot{\boldsymbol{a}}) \boldsymbol{z}_B \right) $$

$\boldsymbol{h}_\omega$ 是将**急动度（Jerk）向量 $\dot{\boldsymbol{a}}$ 投影至飞机的 $\boldsymbol{x}_B$-$\boldsymbol{y}_B$ 平面**（即螺旋桨盘面）得到的向量。该投影向量的大小与方向，可以理解为推力轴的翻转趋势。

将机体角速度在机体坐标系下分解为分量 $p, q, r$，通过 $\boldsymbol{h}_\omega$ 可直接求解前两个分量：
$$
\begin{align*}
p &= -\boldsymbol{h}_\omega \cdot \boldsymbol{y}_B, \\
q &= \boldsymbol{h}_\omega \cdot \boldsymbol{x}_B.
\end{align*}
$$

基于角速度合成关系 $\boldsymbol{\omega}_{BW} = \boldsymbol{\omega}_{BC} + \boldsymbol{\omega}_{CW}$，结合 $\boldsymbol{\omega}_{BC}$ 无 $\boldsymbol{z}_B$ 轴分量的特性，推导第三个分量 $r$：
$$
r = \boldsymbol{\omega}_{CW} \cdot \boldsymbol{z}_B = \dot{\psi} \, \boldsymbol{z}_W \cdot \boldsymbol{z}_B.
$$

#### 3.位置 的四阶导数 (Snap) → 总推力方向的加速度（决定了飞机的角加速度）。
懒得写了，论文里貌似也没有详细展开。
#### 4.角加速度 → 力矩 → 电机的转速差

![alt text](/images/minimum_snap_1.png)

## 2.控制
对照原文和翻译基本可以看懂，可以简单理解为一个PD控制+前馈？

就是优化角度误差那一段比较抽象（因为我线代太拉了）。可以简单理解为：
放弃分别计算三个角度的误差，改为计算当前姿态与目标姿态之间的相对旋转矩阵，并提取出等效旋转矢量（相当于从 **绕三个轴分别转三个角度** 变成了 **绕一个新的轴转一个角度**），确保了修正路径最短。


## 3.轨迹生成
读这一段之前先去把gf的空中机器人重看了一下（悲）。高老师的课主要讲导航和轨迹规划，Minimum snap在快结尾部分出现。

前端工作（路径搜索等）得到的轨迹也是很粗略的，需要后端生成**光滑连续**轨迹，这就是Minimum snap做的工作。

**光滑**通常通过**最小化“输入”的变化率**来实现。
首先明确：四旋翼无人机中，加速度不能突变。

飞机通过的m个关键帧（keyframe）。将整个轨迹分为m段，每段都是关于时间 $t$ 的 $n$ 阶多项式多项式函数。


## 4.术语表
