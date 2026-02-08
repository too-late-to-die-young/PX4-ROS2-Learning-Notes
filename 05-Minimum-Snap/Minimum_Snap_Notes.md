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

![alt text](/PX4-ROS2-Learning-Notes/images/minimum_snap_1.png)

## 2.控制
对照原文和翻译基本可以看懂，可以简单理解为一个PD控制+前馈？

就是优化角度误差那一段比较抽象（因为我线代太拉了）。可以简单理解为：
放弃分别计算三个角度的误差，改为计算当前姿态与目标姿态之间的相对旋转矩阵，并提取出等效旋转矢量（相当于从 **绕三个轴分别转三个角度** 变成了 **绕一个新的轴转一个角度**），确保了修正路径最短。


## 3.轨迹生成
快速建立对minimum_snap的概念可以读这个[终极速通省流版](https://blog.csdn.net/weixin_65874645/article/details/155024581)。

完整学习用的这个[代码](https://github.com/zm0612/Minimum-Snap/tree/2666bbaeb7442b0a3300d93b4ebd487cd8ba26db)和它的[讲解](https://blog.csdn.net/u011341856/article/details/121861930)。

读轨迹生成部分之前先去把gf的空中机器人重看了一下（悲）。高老师的课主要讲导航和轨迹规划，Minimum snap在快结尾部分出现。

前端工作（路径搜索等）得到的轨迹也是很粗略的，需要后端生成**光滑连续**轨迹，这就是Minimum snap做的工作。

### 轨迹的描述
飞机通过m个关键帧（keyframe），将整个轨迹分为m段，每段都是关于时间 $t$ 的 $n$ 阶多项式函数。分段的本质是为了降阶数（如果整个轨迹只用一个不分段的函数表示，它的阶数可能非常高）。

$$
\sigma_T(t) =
\begin{cases}
\displaystyle\sum_{i=0}^n \sigma_{Ti1} t^i & t_0 \leq t < t_1 \\
\displaystyle\sum_{i=0}^n \sigma_{Ti2} t^i & t_1 \leq t < t_2 \\
\vdots & \vdots \\
\displaystyle\sum_{i=0}^n \sigma_{Tim} t^i & t_{m-1} \leq t \leq t_m
\end{cases}
$$

分段的多项式函数到底要取几阶（ $n$ ）取决于你的优化目标是最小化jerk还是snap。

最小化jerk(3)：$3 \times 2 - 1 = 5$

最小化snap(4)：$4 \times 2 - 1 = 7$


### 轨迹优化的目标

轨迹优化的最终目标——光滑，通常通过**最小化“输入”的变化率**来实现：

$$
\min \int_{t_0}^{t_m} \left[ \mu_r \left\| \frac{d^{k_r} \boldsymbol{r}_T}{dt^{k_r}} \right\|^2 + \mu_\psi \left( \frac{d^{k_\psi} \psi_T}{dt^{k_\psi}} \right)^2 \right] dt
$$

两项分别对应**平移运动**和**偏航运动**：
- **平移运动**（位置 $\boldsymbol{r}$），取 $k_r=4$ ，因为控制输入（电机力矩/推力的导数）与位置的4阶导数相关。
- **偏航角**（Yaw $\psi$），取 $k_\psi=2$ ，因为偏航力矩与偏航角的2阶导数相关。

我们对两项分别平方（有点像求方差），一方面避免正负值相互抵消；同时构造二次规划（QP）问题。

### 约束条件

在达到优化目标的同时，轨迹还需要满足一些约束条件。一个最简单的例子如下（不考虑飞行走廊，最大速度等约束）：

$$
\sigma_T(t_i) = \sigma_i, \quad i = 0, \dots, m
$$

$$
\left. \frac{d^p x_T}{dt^p} \right|_{t=t_j} = 0 \ \text{or free}, \quad j = 0, m; \ p = 1, \dots, k_r
$$

$$
\left. \frac{d^p y_T}{dt^p} \right|_{t=t_j} = 0 \ \text{or free}, \quad j = 0, m; \ p = 1, \dots, k_r
$$

$$
\left. \frac{d^p z_T}{dt^p} \right|_{t=t_j} = 0 \ \text{or free}, \quad j = 0, m; \ p = 1, \dots, k_r
$$

$$
\left. \frac{d^p \psi_T}{dt^p} \right|_{t=t_j} = 0 \ \text{or free}, \quad j = 0, m; \ p = 1, \dots, k_\psi
$$

在这个例子中，我们规定了无人机始末位置的全部运动状态（位置，速度，加速度，jerk，snap）；以及无人机途径的所有点（仅确定位置）。

四旋翼无人机中，加速度不能突变。所以还需要引入一个连续性约束（也就是两段轨迹之间的衔接要高阶连续）：

*在关键帧时刻 $t_1,\dots,t_{m-1}$，位置 $\boldsymbol{r}_T$ 的前 $k_r$ 阶导数和偏航角 $\psi_T$ 的前 $k_\psi$ 阶导数保持连续。（取 $k_r=4$ $k_\psi=2$ ）*

### 问题的一般化建模

上文介绍了需要解决的问题，但是得到的优化目标和约束都很复杂。这个部分的目标是把上述复杂的目标和约束转化成一个简洁的、方便投给电脑求解的形式。
（以下让gemini概括的，线代白痴很绝望了）
##### 变量向量化 (Vectorization)
将所有段、所有轴的多项式系数拼成一个巨大的向量 $\boldsymbol{c}$。对于每一段 $i$，其系数为 $\boldsymbol{c}_i=[a_{i0},a_{i1},\dots,a_{in}]^T$。整个问题的决策变量就是：
$$
\boldsymbol{c}=[\boldsymbol{c}_1^T,\boldsymbol{c}_2^T,\dots,\boldsymbol{c}_m^T]^T
$$

##### 构建 Hessian 矩阵 $\boldsymbol{H}$ (Cost Mapping)
把目标函数（积分式）改写为标准二次型：$\min \boldsymbol{c}^T \boldsymbol{H} \boldsymbol{c}$。

- 单段代价：对于第 $i$ 段，其 $k$ 阶导数平方的积分可以表示为 $\boldsymbol{c}_i^T \boldsymbol{Q}_i \boldsymbol{c}_i$。其中 $\boldsymbol{Q}_i$ 矩阵的元素是通过对 $t$ 的幂函数求导再积分得到的。
- 全段集成：将每一段的 $\boldsymbol{Q}_i$ 按照块对角线排列，就得到了总的 Hessian 矩阵 $\boldsymbol{H}$。
- 注意：$\boldsymbol{H}$ 是半正定的，这保证了问题的凸性（有一个很好的特性：局部最优即为全局最优）。

##### 构建约束矩阵 $\boldsymbol{A}$ (Constraint Mapping)
把所有的等式约束（起点终点状态、经过点位置、连续性要求）写成 $\boldsymbol{A}_{eq} \boldsymbol{c} = \boldsymbol{b}_{eq}$ 的形式。

 物理限制（不等式约束）：$\boldsymbol{A}_{ieq} \boldsymbol{c} \leq \boldsymbol{b}_{ieq}$，比如速度不能超过 $v_{\text{max}}$，轨迹必须在走廊 $\delta$ 内。

##### 最终的 QP 标准型
现在，复杂的轨迹优化问题被转化为标准的二次规划（QP）问题，可直接调用求解器求解：
$$
\begin{cases}
\min\limits_{\boldsymbol{c}} & \boldsymbol{c}^T \boldsymbol{H} \boldsymbol{c} + \boldsymbol{f}^T \boldsymbol{c} \\
\text{s.t.} & \boldsymbol{A}_{eq} \boldsymbol{c} = \boldsymbol{b}_{eq} \\
& \boldsymbol{A}_{ieq} \boldsymbol{c} \leq \boldsymbol{b}_{ieq}
\end{cases}
$$
这个写完还是太抽象了，我打算手搓一个示例po在后面



### 引入更复杂的情景
在实际飞行中，还需要解决数值计算稳定性、空间安全性以及时间分配的合理性问题。

#### 无量纲化/时空缩放
由于轨迹通常涉及高阶多项式（如 7 阶），在计算过程中会出现 $t^7$ 甚至更高的项，在实际计算中可能引入极大或极小的数值。因此，我们将飞行时间归一化。

**时间缩放**  
引入无量纲时间 $\tau\in[0,1]$，通过时间映射公式 $t=\alpha\tau$（其中 $\alpha$ 叫时间缩放因子，为该段轨迹的总时长），将每段轨迹的时间维度归一化。
原时间域的多项式 $w(t)$ 转换为无量纲时间域的多项式 $\tilde{w}(\tau)$。


**空间缩放**
引入空间缩放因子 $\beta_2$ 和平移因子 $\beta_1$，定义有量纲物理空间变量 $w(t)$ 与无量纲标准空间变量 $\tilde{w}(\tau)$ 的线性关系：
$$w(t)=w(\alpha\tau)=\beta_1+\beta_2\tilde{w}(\tau)$$

计算缩放前后的代价函数，发现它们只差一个系数：
$$\int_{0}^{\alpha} \left\| \frac{d^k w(t)}{dt^k} \right\|^2 dt = \alpha^{2k-1}\beta_2^2 \int_{0}^{1} \left\| \frac{d^k \tilde{w}(\tau)}{d\tau^k} \right\|^2 d\tau$$

系数 $\alpha^{2k-1}\beta_2^2$ 是与轨迹无关的常数，这意味着缩放前后最优解不变！这就带来了一些很好的性质：

- 若路径形状不变，仅改变起终点的跨度，可通过线性变换系数 $\beta$ 直接缩放多项式系数，无需重新求解 QP 问题。

- 若飞行过程中出现速度/加速度超限，只需增大时间因子 $\alpha$，轨迹的几何形状保持不变，但位置的所有阶导数会按比例下降，保证飞行安全。

#### 飞行走廊约束
在现实场景中无人机有导航或避障的需求，必须在安全的飞行走廊内通过。通过线性化、采样，将这个问题转化为一个不等式约束。

**线性化**
在相邻关键帧 $\boldsymbol{r}_i$ 到 $\boldsymbol{r}_{i+1}$ 的连线上，定义轨迹相对于该连线的垂直偏移量 $\boldsymbol{d}_i(t)$。
$$
\boldsymbol{d}_i(t) = \left( \boldsymbol{r}_T(t) - \boldsymbol{r}_i \right) - \left( \left( \boldsymbol{r}_T(t) - \boldsymbol{r}_i \right) \cdot \boldsymbol{t}_i \right) \boldsymbol{t}_i
$$

**采样**
在每段轨迹的时间域内均匀选取 $n_c$ 个采样点，强制所有采样点的偏移量落在走廊宽度 $\delta_i$ 内：
$$\|\boldsymbol{d}_i(t_j)\|_\infty \leq \delta_i$$
其中 $t_j$ 为第 $j$ 个采样点对应的时刻，$\|\cdot\|_\infty$ 为无穷范数（切比雪夫范数），可保证偏移量在各空间轴向上的绝对值均不超过安全阈值。

这样就得到了 $2\times n_c$ 个易处理的线性不等式约束，可以放到二次规划的标准约束形式 $\boldsymbol{A}_{ieq}\boldsymbol{c} \leq \boldsymbol{b}_{ieq}$ 里面。

#### 最优航段时间
分段轨迹取决于分段时间分配，时间分配显著影响最终轨迹。这个部分解决如何合理分配时间的问题。

可以先这样理解：

我们要找到最优时间分配方案（代价函数最小）：
$$
\min \quad f(\boldsymbol{T})
$$


先“随便”给每段分配个初始时长：
$$
\boldsymbol{T} = [T_1, T_2, \dots, T_m]
$$

$$
 \quad \sum T_i = t_m
$$

$$
T_i \geq 0
$$

对于每一段时间，我们微微 增加/减小 它的长度，并把增减量平均扣除到剩下的时长里（使得总时长保持不变），观察总代价函数是变大了还是变小了。始终往代价变小的方向调。

这个微调各段时长的操作，写成一个向量 $g_i$ ，它表示 "增加第 $i$ 段的时间，同时同比例减少其他段的时间" 。

这个微调带来的代价函数变化结果则用梯度表示：

$$
\nabla_{g_i} f = \frac{f(\boldsymbol{T} + h g_i) - f(\boldsymbol{T})}{h}
$$



处理完一段时间后接着处理下一段，处理完一轮后接着处理下一轮……直到算法收敛。



我觉得这是这里面最抽象的一个东西（


## 4.术语表
叫ai做的，没细审过
| 中文术语 | 英文术语 |
| ---- | ---- |
| 高度受限场景 | tightly constrained setting |
| 控制器设计 | controller design |
| 轨迹生成 | trajectory generation |
| 横滚角 | roll angle |
| 俯仰角 | pitch angle |
| 小角度近似 | small angle approximation |
| 偏航角 | yaw angle |
| 最优轨迹 | optimal trajectory |
| 飞行走廊 | corridor |
| 速度 | velocity |
| 加速度 | acceleration |
| 输入量 | input |
| 非线性控制器 | nonlinear controller |
| 微型无人机 | micro Unmanned Aerial Vehicle (micro UAV) |
| 四旋翼无人机 | quadrotor |
| 模型线性化 | model linearization |
| 动态系统 | dynamic system |
| 可达性算法 | reachability algorithm |
| 增量搜索技术 | incremental search technique |
| 线性二次调节器树搜索算法 | LQR-tree-based search algorithm |
| 状态空间 | state space |
| 特技飞行动作 | aerobatic maneuver |
| 稳定性 | stability |
| 收敛性 | convergence |
| 机器学习技术 | machine learning technique |
| 强化学习 | reinforcement learning |
| 运动规划 | motion planning |
| 模型预测控制 | Model Predictive Control (MPC) |
| 线性化模型 | linearized model |
| 控制李雅普诺夫函数 | control Lyapunov function |
| 飞行规划 | flight plan |
| 动力学特性 | dynamics |
| 可达速度 | achievable velocity |
| 关键帧 | keyframe |
| 坐标系 | coordinate system |
| 世界坐标系 | world frame |
| 机体坐标系 | body frame |
| 螺旋桨编号规则 | propeller numbering convention |
| 奇异性 | singularity |
| 旋转矩阵 | rotation matrix |
| Z-X-Y欧拉角 | Z-X-Y Euler angles |
| 角速度 | angular velocity |
| 螺旋桨 | propeller |
| 升力 | force |
| 力矩 | moment |
| 电机动力学特性 | motor dynamics |
| 刚体动力学 | rigid body dynamics |
| 空气动力学特性 | aerodynamics |
| 总升力 | net body force |
| 机体力矩 | body moment |
| 质心 | center of mass |
| 位置矢量 | position vector |
| 重力 | gravity |
| 牛顿运动定律 | Newton’s equations of motion |
| 角加速度 | angular acceleration |
| 欧拉方程 | Euler equations |
| 转动惯量矩阵 | moment of inertia matrix |
| 状态量 | state |
| 微分平坦系统 | differentially flat system |
| 平坦输出量 | flat output |
| 欠驱动系统 | underactuated system |
| 位置误差 | position error |
| 速度误差 | velocity error |
| 期望力向量 | desired force vector |
| 正定增益矩阵 | positive definite gain matrix |
| 姿态误差 | orientation error |
| 向量映射算子 | vee map |
| 特殊正交李代数 | Special Orthogonal Lie Algebra (so(3)) |
| 角速度误差 | angular velocity error |
| 对角增益矩阵 | diagonal gain matrix |
| 前馈项 | feedforward term |
| 初始条件 | initial condition |
| 指数稳定性 | exponential stability |
| 几乎全局指数吸引性 | almost globally exponential attractiveness |
| 分段多项式函数 | piecewise polynomial function |
| 代价函数 | cost function |
| 优化问题 | optimization problem |
| 无量纲化常数 | nondimensional constant |
| 二次规划问题 | Quadratic Program (QP) |
| 决策变量 | decision variable |
| 无量纲化 | nondimensionalization |
| 无量纲变量 | nondimensional variable |
| 时间缩放 | temporal scaling |
| 空间缩放 | spatial scaling |
| 垂直距离向量 | perpendicular distance vector |
| 无穷范数 | infinity norm |
| 线性约束 | linear constraint |
| 最优航段时间 | optimal segment time |
| 约束梯度下降法 | constrained gradient descent method |
| 方向导数 | directional derivative |
| 回溯线搜索法 | backtracking line search |
| 运动捕捉系统 | motion capture system |
| 陀螺仪 | gyro |
| 二次空气阻力模型 | quadratic air drag model |
| 甩动 | snap |
| 加加速度 | jerk |
| 特殊正交群 | Special Orthogonal Group (SO(2)/SO(3)) |
| 实数空间 | Real Space ($\mathbb{R}^{3}$) |
| 迹 | Trace (tr) |
| 最小特征值 | Minimum Eigenvalue ($\lambda_{min}$) |
| 单位向量 | Unit Vector |
| 点乘 | Dot Product (·) |
| 叉乘 | Cross Product (×) |
| 多项式基函数 | Polynomial Basis Function |
| 梯度 | Gradient |
| 积分 | Integral |
| 导数 | Derivative |
| 等式约束 | Equality Constraint |
| 不等式约束 | Inequality Constraint |
| 目标函数 | Objective Function |