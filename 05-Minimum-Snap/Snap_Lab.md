# 一个手搓尝试

推荐阅读：

快速建立对minimum_snap的概念可以读这个[终极速通省流版](https://blog.csdn.net/weixin_65874645/article/details/155024581)。

完整学习用的这个[代码](https://github.com/zm0612/Minimum-Snap/tree/2666bbaeb7442b0a3300d93b4ebd487cd8ba26db)和它的[讲解](https://blog.csdn.net/u011341856/article/details/121861930)。或[这个仓库](https://github.com/symao/minimum_snap_trajectory_generation?tab=readme-ov-file)。


## 0.准备
安一个miniconda
我一开始准备的是3.10的python环境，被坑死了，记得选ROS版本适配的python
```bash
conda create -n snap_env python=3.12

# 进入环境
conda activate snap_env

# 若要退出
conda deactivate
```

在snap_env的环境下：
```bash
pip install numpy sympy matplotlib osqp
```

## 1. 平面一维轨迹生成
新建一个python文件，把代码复制进去跑就行了(代码放在`scripts/minimum_snap_lab.py`)。

在snap_env环境下运行代码：
```bash
python $文件绝对路径
```
### 代码解释

构造单段轨迹的代价矩阵：
```py
def get_symbolic_q(n,r):

    #n: 多项式阶数 (7阶) r: 优化目标 (4阶导Snap最小)

    # 定义符号变量(t, T)和系数c0, c1, ..., cn
    t, T = sp.symbols('t T')
    c = sp.symbols(f'c0:{n+1}')

    # 定义多项式轨迹
    p = sum(c[i] * (t**i) for i in range(n+1))

    # 计算r阶导数
    p_der = sp.diff(p,t,r)
    
    #最小化能量泛函（这段代码表示以t为积分变量，0为积分下限，T为积分上限，对p_der的平方积分）
    energy = sp.integrate(p_der**2,(t,0,T))

    # 提取二次项系数矩阵Q
    Q_sym = sp.hessian(energy,c)

    # 将符号矩阵转化为可用的 NumPy 函数，方便后续填入具体的 T 值
    q_func = sp.lambdify(T,Q_sym, 'numpy')

    return Q_sym, q_func
```
将代价矩阵对角线拼合：
```py
def get_global_Q(T_list,q_func):

    #假设有m段轨迹，每段飞行时间存放在T_list里
    # q_func则是上面构建的求q矩阵的函数
    
    Q_list = [q_func(t) for t in T_list]
    
    #使用block_diag对角拼接
    Q_all = block_diag(*Q_list)

    return Q_all
```

```py
# 创建多项式在时间 t 处的r阶导数的基向量
def get_poly_basis(t,n,r):
    basis = np.zeros(n + 1)
    for i in range(r, n+1):
        #计算 i*(i-1)*...*(i-r+1)
        val = np.prod([i - j for j in range(r)])
        basis[i] = val * (t**(i - r))
    return basis
```

约束矩阵中的位置约束(注意每段轨迹的始末位置都要放一次，所以中间的位置点都是重复出现两次的，一次作为首，一次作为尾)：
```py
    for i in range(M):
        #轨迹起点
            #创建一个元素权全为0，长度为num_params的数组
        row_start = np.zeros(num_params) 
        row_start[i*(n+1) : (i+1)*(n+1)] = get_poly_basis(0,n,0)
        A_eq.append(row_start)
        b_eq.append(waypoints[i])

        #轨迹终点
        row_end = np.zeros(num_params)
        row_end[i*(n+1) : (i+1)*(n+1)] = get_poly_basis(T_list[i], n, 0)
        A_eq.append(row_end)
        b_eq.append(waypoints[i+1])
```
连续性约束，注意这里使用局部时间参数，每段轨迹时间从0开始计算，而T_list中存放的是每段时长：
```py
    for i in range(M - 1):
        for r in range(1, 4):
            row = np.zeros(num_params)
            # 第 i 段的终点导数
            row[i*(n+1) : (i+1)*(n+1)] = get_poly_basis(T_list[i], n, r)
            # 减去第 i+1 段的起点导数
            row[(i+1)*(n+1) : (i+2)*(n+1)] = -get_poly_basis(0, n, r)
            A_eq.append(row)
            b_eq.append(0)
```
后面就是求解器、时间分配和可视化绘图。

### 优化
#### 时间分配优化
如果使用均分的时间分配方案，每段时间相等，生成轨迹：
![alt text](/images/snap_lab_1.png)

使用梯形时间分配：
```py
def allocate_time(waypoints, v_max, a_max):
    """
    根据梯形速度剖面分配每段轨迹的时间
    但是这里算时间用的都是直线，所以最大速度最好留一点余量
    """
    T_list = []
    # 计算加速到最大速度所需的位移
    # v = a*t -> t = v/a
    # d = 0.5 * a * t^2 = 0.5 * v^2 / a
    d_acc_dec = (v_max**2) / (a_max) # 加速段+减速段的总位移
    
    for i in range(len(waypoints) - 1):
        dist = abs(waypoints[i+1] - waypoints[i])
        
        if dist > d_acc_dec:
            # 情况1：能达到最大速度 (梯形)
            t_acc = v_max / a_max
            t_const = (dist - d_acc_dec) / v_max
            T = 2 * t_acc + t_const
        else:
            # 情况2：达不到最大速度 (三角形)
            # dist = 2 * (0.5 * a * t_half^2) = a * t_half^2
            # t_half = sqrt(dist / a)
            T = 2 * np.sqrt(dist / a_max)
            
        # 避免时间过短导致数值问题
        T_list.append(max(T, 0.1))
        
    return T_list
```
生成轨迹：
![alt text](/images/snap_lab_2.png)

#### 闭式求解

如果QP问题只有等式约束，没有不等式约束，则是可以闭式求解的。这样效率要高得多。
参考论文：
*Polynomial Trajectory Planning for Aggressive
Quadrotor Flight in Dense Indoor Environments*

原本的求解器：
```python
def solve_osqp(Q_all, A_eq, b_eq):
    # 1. 准备目标函数矩阵 P (即 Q_all)
    # OSQP 要求 P 是稀疏矩阵且为上三角或全矩阵
    P = sparse.csc_matrix(Q_all)
    q = np.zeros(Q_all.shape[0])

    # 2. 准备约束矩阵 A
    # OSQP 的约束形式是 l <= Ax <= u
    A = sparse.csc_matrix(A_eq)
    
    # 对于等式约束，下限 l 和上限 u 相等，都等于 b_eq
    l = b_eq
    u = b_eq

    # 3. 创建 OSQP 实例并配置
    prob = osqp.OSQP()
    
    # setup 函数初始化问题
    # eps_abs/rel 是收敛精度，对于轨迹规划可以设得细一点
    prob.setup(P, q, A, l, u, warm_start=True, verbose=False, eps_abs=1e-8, eps_rel=1e-8)
    
    # 4. 求解
    res = prob.solve()
    
    # 检查状态并返回结果
    if res.info.status == 'solved':
        return res.x
    else:
        raise ValueError("OSQP 未能找到最优解，请检查约束是否冲突。")

```
如果使用闭式求解：
```python
def minimum_snap_closed_form(wayp, ts, n_order, v0, a0, v1, a1):
    # n_order轨迹次数， n_coef多项式系数数， n_poly 轨迹段数， n_continuous轨迹连续次数
    n_coef = n_order + 1
    n_poly = len(wayp) - 1
    n_continuous = 4  

    # 1. compute Q (直接利用你原本的符号 Q 函数逻辑)
    _, q_func = get_symbolic_q(n_order, 4) 
    Q_all = block_diag(*[q_func(T) for T in (np.diff(ts))])

    # 2. compute A (由多项式系数映射到段端点导数)
    # 结构完全参考 Matlab: A * p = d_segments
    A = np.zeros((n_continuous * 2 * n_poly, n_coef * n_poly))
    for i in range(n_poly):
        T_seg = ts[i+1] - ts[i]
        for j in range(n_continuous):
            # 起点 t=0
            A[n_continuous*2*i + j, n_coef*i : n_coef*(i+1)] = get_poly_basis(0, n_order, j)
            # 终点 t=T_seg (注意：这里使用局部时间)
            A[n_continuous*2*i + n_continuous + j, n_coef*i : n_coef*(i+1)] = get_poly_basis(T_seg, n_order, j)
    
    # 3. compute M (将每段独立的端点导数 映射到 唯一的节点导数)
    num_d = n_continuous * (n_poly + 1)
    M = np.zeros((n_poly * 2 * n_continuous, num_d))
    for i in range(n_poly):
        # 第 i 段起点 -> 第 i 个节点
        M[2*i*n_continuous : (2*i+1)*n_continuous, i*n_continuous : (i+1)*n_continuous] = np.eye(n_continuous)
        # 第 i 段终点 -> 第 i+1 个节点
        M[(2*i+1)*n_continuous : (2*i+2)*n_continuous, (i+1)*n_continuous : (i+2)*n_continuous] = np.eye(n_continuous)

    # 4. compute C (置换矩阵，重排固定导数和自由导数)
    # fix_idx 包含：所有位置(p)，起点v,a，终点v,a
    fix_idx = []
    for i in range(n_poly + 1):
        fix_idx.append(i * n_continuous) # 所有节点的位置 p
    fix_idx.extend([1, 2]) # 起点 v, a
    fix_idx.extend([num_d - n_continuous + 1, num_d - n_continuous + 2]) # 终点 v, a
    fix_idx = sorted(list(set(fix_idx)))
    free_idx = [i for i in range(num_d) if i not in fix_idx]

    # 构建 df (固定导数值)
    df = np.zeros(len(fix_idx))
    for i, idx in enumerate(fix_idx):
        if idx % n_continuous == 0:
            df[i] = wayp[idx // n_continuous]
        # v0, a0, v1, a1 已默认为 0

    # 组合 C 矩阵并计算 R
    # eye：生成单位矩阵
    C = np.eye(num_d)[:, fix_idx + free_idx]
    
    # 这里的 AiMC 对应 Matlab 里的 inv(A)*M*C
    AiMC = inv(A) @ M @ C
    R = AiMC.T @ Q_all @ AiMC

    # 5. 矩阵分块求解
    n_fix = len(fix_idx)
    Rff = R[:n_fix, :n_fix]
    Rfp = R[:n_fix, n_fix:]
    Rpf = R[n_fix:, :n_fix]
    Rpp = R[n_fix:, n_fix:]

    # 最优解公式: dp = -inv(Rpp) * Rpf * df
    dp = -inv(Rpp) @ Rpf @ df

    # 恢复系数 p = AiMC * [df; dp]
    p_all = AiMC @ np.concatenate([df, dp])
    polys = p_all.reshape(n_poly, n_coef)
    return polys
```
## 2. 三维轨迹生成和飞行走廊约束

代码参考`scripts/minimum_snap_lab_3d.py`
未加飞行走廊约束：
![alt text](/images/snap_lab_3.png)
加了（全段轨迹都加）：
![alt text](/images/snap_lab_4.png)
加了中间一段（这个叫ai写的，随便跑了一下，代码没有存在这里）：
![alt text](/images/snap_lab_5.png)


## 3.rviz可视化
在你的工作空间src目录下：
```bash
ros2 pkg create --build-type ament_python trajectory_viz --dependencies rclpy visualization_msgs geometry_msgs
```
`--build-type ament_python`：指定开发语言为python
`trajectory_viz`：包名
`--dependencies `：我的包需要哪些外部库，这样就会自动写到`package.xml`里面

把可视化的python的代码导到`trajectory_viz/trajectory_viz`里。代码在`/scripts/snap_rviz_visualize.py`。
conda环境里不能`ros2 run`，要直接`python `运行。

可视化了飞行走廊和轨迹之后果真发现一个bug，走廊采样点太稀疏了，并没有全覆盖轨迹，所以把采样改得稍微密了一点。
成品：
![alt text](/images/snap_lab_6.png)