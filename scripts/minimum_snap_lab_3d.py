import sympy as sp
import numpy as np
from scipy.linalg import block_diag
import osqp
from scipy import sparse
import matplotlib.pyplot as plt

def get_symbolic_q(n,r):
    t, T = sp.symbols('t T')
    c = sp.symbols(f'c0:{n+1}')
    p = sum(c[i] * (t**i) for i in range(n+1))
    p_der = sp.diff(p,t,r)
    energy = sp.integrate(p_der**2,(t,0,T))
    Q_sym = sp.hessian(energy,c)
    q_func = sp.lambdify(T,Q_sym, 'numpy')
    return Q_sym, q_func

def get_global_Q(T_list,q_func):
    Q_list = [q_func(t) for t in T_list]
    Q_all = block_diag(*Q_list)
    return Q_all

def get_poly_basis(t,n,r):
    basis = np.zeros(n + 1)
    for i in range(r, n+1):
        val = np.prod([i - j for j in range(r)])
        basis[i] = val * (t**(i - r))
    return basis

def build_constraints(waypoints, T_list, n=7):
    M = len(T_list)
    num_params = M * (n+1)
    A_eq = []
    b_eq = []

    for i in range(M):
        row_start = np.zeros(num_params) 
        row_start[i*(n+1) : (i+1)*(n+1)] = get_poly_basis(0,n,0)
        A_eq.append(row_start)
        b_eq.append(waypoints[i])

        row_end = np.zeros(num_params)
        row_end[i*(n+1) : (i+1)*(n+1)] = get_poly_basis(T_list[i], n, 0)
        A_eq.append(row_end)
        b_eq.append(waypoints[i+1])

    for i in range(M - 1):
        for r in range(1, 4):
            row = np.zeros(num_params)
            row[i*(n+1) : (i+1)*(n+1)] = get_poly_basis(T_list[i], n, r)
            row[(i+1)*(n+1) : (i+2)*(n+1)] = -get_poly_basis(0, n, r)
            A_eq.append(row)
            b_eq.append(0)

    for r in [1, 2]:
        row_start_boundary = np.zeros(num_params)
        row_start_boundary[0 : n+1] = get_poly_basis(0, n, r)
        A_eq.append(row_start_boundary)
        b_eq.append(0)

        row_end_boundary = np.zeros(num_params)
        row_end_boundary[-(n+1):] = get_poly_basis(T_list[-1], n, r)
        A_eq.append(row_end_boundary)
        b_eq.append(0)

    return np.array(A_eq), np.array(b_eq)

def solve_osqp(Q_all, A_final, l_final, u_final):
  
    P = sparse.csc_matrix(Q_all)
    q = np.zeros(Q_all.shape[0])

    # 2. 准备约束矩阵 A
    # OSQP 的约束形式是 l <= Ax <= u
    A = sparse.csc_matrix(A_final)
    
    # 对于等式约束，下限 l 和上限 u 相等，都等于 b_eq
    l = l_final
    u = u_final

    prob = osqp.OSQP()
    
    prob.setup(P, q, A, l, u, warm_start=True, verbose=False, eps_abs=1e-8, eps_rel=1e-8)
    
    res = prob.solve()

    if res.info.status == 'solved':
        return res.x
    else:
        raise ValueError("OSQP 未能找到最优解，请检查约束是否冲突。")


class MinimumSnapPlanner3D:
    def __init__(self, n_order=7, v_max=2.0, a_max=1.5):
        self.n = n_order
        self.v_max = v_max
        self.a_max = a_max
        # 1. 预生成 Q 矩阵的函数 (Symbolic -> Numpy)
        _, self.q_func = get_symbolic_q(n=self.n, r=4)

    def plan(self, waypoints_3d, corridor_width):
        """
        waypoints_3d: np.array([[x0,y0,z0], [x1,y1,z1], ...])
        """
        # --- 第一步：时间分配 (三轴共用) ---
        # 参考一维的 allocate_time，但输入距离改为 3D 欧集距离
        T_list = self._allocate_time_3d(waypoints_3d)
        
        # --- 第二步：准备全局代价矩阵 Q ---
        Q_all = get_global_Q(T_list, self.q_func)
        
        # --- 第三步：分轴求解 ---
        # 三个轴的时间、Q矩阵是一样的，但路点约束不同
        coeffs_3d = {
            'x': None,
            'y': None,
            'z': None
        }
        
        for i, axis in enumerate(['x', 'y', 'z']):
            axis_waypoints = waypoints_3d[:, i]
            
            # 1. 构建该轴的等式约束 (位置、连续性、边界)
            A_eq, b_eq = build_constraints(axis_waypoints, T_list, n=self.n)
            
            # 2. (可选) 构建该轴的不等式约束 (飞行走廊)
            if corridor_width is not None:
                A_ineq, l_ineq, u_ineq = self._build_corridor_constraints(
                    axis_waypoints, T_list, corridor_width
                )
                # 拼接等式与不等式
                A_final = np.vstack([A_eq, A_ineq])
                l_final = np.hstack([b_eq, l_ineq])
                u_final = np.hstack([b_eq, u_ineq])
            else:
                A_final, l_final, u_final = A_eq, b_eq, b_eq
            
            # 3. 调用 OSQP 求解该轴的系数
            coeffs_3d[axis] = solve_osqp(Q_all, A_final, l_final, u_final)
            
        return coeffs_3d, T_list

    def _allocate_time_3d(self, waypoints_3d):
        # 填入你之前的 allocate_time 逻辑
        # 距离 dist = np.linalg.norm(waypoints_3d[i+1] - waypoints_3d[i])
        T_list = []
        d_acc_dec = (self.v_max**2) / (self.a_max)

        for i in range(len(waypoints_3d) - 1):
            dist = np.linalg.norm(waypoints_3d[i+1] - waypoints_3d[i])
            if dist > d_acc_dec:
            # 情况1：能达到最大速度 (梯形)
                t_acc = self.v_max / self.a_max
                t_const = (dist - d_acc_dec) / self.v_max
                T = 2 * t_acc + t_const
            else:
            # 情况2：达不到最大速度 (三角形)
            # dist = 2 * (0.5 * a * t_half^2) = a * t_half^2
            # t_half = sqrt(dist / a)
                T = 2 * np.sqrt(dist / self.a_max)
            
        # 避免时间过短导致数值问题
            T_list.append(max(T, 0.2))
        
        return T_list


    def _build_corridor_constraints(self, axis_waypoints, T_list, width):
        # 填入不等式约束逻辑：在每段轨迹中间采样 t_check
        # l = center - width, u = center + width
        # A_row = get_poly_basis(t_check, n, 0)
        #在xyz三个平面上分别约束，最后将飞机置于一个方管形的走廊内（圆管计算过于复杂了）
        A_eq = []
        l_eq = []
        u_eq = []
        M = len(T_list)
        num_params = M * (self.n + 1)
        for i in range(M):
            # 1/3处采样
            for fraction in [0.33, 0.66]:
                t_check = fraction * T_list[i]
                A_row = np.zeros(num_params)
                A_row[i*(self.n + 1) : (i + 1)*(self.n + 1)] = get_poly_basis(t_check, self.n, 0)
                
                center = axis_waypoints[i] + fraction * (axis_waypoints[i+1] - axis_waypoints[i])
                #不用欧式距离！！这样更好算。。
                l = center - width
                u = center + width

                A_eq.append(A_row)
                l_eq.append(l)
                u_eq.append(u)
                
        return np.array(A_eq), np.array(l_eq), np.array(u_eq)

# --- 使用示例 ---
if __name__ == "__main__":
    # 1. 定义空间路点
    path = np.array([
        [0, 0, 0],
        [2, 4, 2],
        [4, 0, 4]
    ])
    
    # 2. 规划
    planner = MinimumSnapPlanner3D()
    all_coeffs, times = planner.plan(path, corridor_width= 0.4)
    
    # 3. 采样并输出给仿真器或绘图
    # 逻辑：遍历 times -> 遍历 t_step -> 组合 x,y,z 系数 -> 计算位置
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    total_t = 0
    for i in range(len(times)):
        t_plot = np.linspace(0, times[i], 50)
        points = np.zeros((50, 3))
        for j, axis in enumerate(['x', 'y', 'z']):
            c_seg = all_coeffs[axis][i*(planner.n + 1):(i+1)*(planner.n+1)]
            points[:, j] = [np.dot(c_seg, get_poly_basis(tp, 7, 0)) for tp in t_plot]
        ax.plot(points[:, 0], points[:, 1], points[:, 2], label=f'Seg {i+1}')

    ax.scatter(path[:,0], path[:,1], path[:,2], c='r', s=50, label='Waypoints')
    ax.set_title("3D Minimum Snap with Box Corridor")
    plt.legend(); plt.show()