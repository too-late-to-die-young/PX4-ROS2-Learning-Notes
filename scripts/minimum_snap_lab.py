import sympy as sp
import numpy as np
from scipy.linalg import block_diag
import osqp
from scipy import sparse
import matplotlib.pyplot as plt

#单段轨迹的代价矩阵
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

# 构建全局代价矩阵
def get_global_Q(T_list,q_func):

    #假设有m段轨迹，每段飞行时间存放在T_list里
    # q_func则是上面构建的求q矩阵的函数
    
    Q_list = [q_func(t) for t in T_list]
    
    #使用block_diag对角拼接
    Q_all = block_diag(*Q_list)

    return Q_all

# 创建多项式在时间 t 处的r阶导数的基向量
def get_poly_basis(t,n,r):
    basis = np.zeros(n + 1)
    for i in range(r, n+1):
        #计算 i*(i-1)*...*(i-r+1)
        val = np.prod([i - j for j in range(r)])
        basis[i] = val * (t**(i - r))
    return basis

# 约束矩阵
def build_constraints(waypoints, T_list, n=7):
    
    M = len(T_list)
    num_params = M * (n+1)

    A_eq = []
    b_eq = []
    
    #位置约束
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

    #连续性约束
    #注意这里使用局部时间参数，每段轨迹时间从0开始计算
    for i in range(M - 1):
        for r in range(1, 4):
            row = np.zeros(num_params)
            # 第 i 段的终点导数
            row[i*(n+1) : (i+1)*(n+1)] = get_poly_basis(T_list[i], n, r)
            # 减去第 i+1 段的起点导数
            row[(i+1)*(n+1) : (i+2)*(n+1)] = -get_poly_basis(0, n, r)
            A_eq.append(row)
            b_eq.append(0)

    #边界约束（起终点速度加速度，这里我们设为0）
    for r in [1, 2]:
        # 起点
        row_start_boundary = np.zeros(num_params)
        row_start_boundary[0 : n+1] = get_poly_basis(0, n, r)
        A_eq.append(row_start_boundary)
        b_eq.append(0)

        # 终点
        row_end_boundary = np.zeros(num_params)
        row_end_boundary[-(n+1):] = get_poly_basis(T_list[-1], n, r)
        A_eq.append(row_end_boundary)
        b_eq.append(0)

    return np.array(A_eq), np.array(b_eq)

# 求解器
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


n = 7
waypoints = [0, 5, 2, 8] # 一维路点
T_list = allocate_time(waypoints, v_max=2.0, a_max=1.5)

q_func = get_symbolic_q(n=n, r=4)[1]
Q_all = block_diag(*[q_func(t) for t in T_list])
A_eq, b_eq = build_constraints(waypoints, T_list, n=n)
coeffs = solve_osqp(Q_all, A_eq, b_eq)

# --- 绘图检查 ---
time_log, pos_log = [], []
for i in range(len(T_list)):
    t_vals = np.linspace(0, T_list[i], 100)
    c_seg = coeffs[i*(n+1) : (i+1)*(n+1)]
    for tv in t_vals:
        time_log.append(tv + sum(T_list[:i]))
        pos_log.append(np.dot(c_seg, get_poly_basis(tv, n, 0)))

plt.plot(time_log, pos_log, label='Trajectory')
plt.scatter(np.cumsum([0]+T_list), waypoints, color='red', label='Waypoints')
plt.title("1D Minimum Snap Trajectory")
plt.legend(); plt.grid(True); plt.show()