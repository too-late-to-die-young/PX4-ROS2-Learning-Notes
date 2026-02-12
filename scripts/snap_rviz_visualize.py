
import rclpy
from rclpy.node import Node
import sympy as sp
import numpy as np
from scipy.linalg import block_diag
import osqp
from scipy import sparse
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

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

    A = sparse.csc_matrix(A_final)
    
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
       
        _, self.q_func = get_symbolic_q(n=self.n, r=4)

    def plan(self, waypoints_3d, corridor_width):
        """
        waypoints_3d: np.array([[x0,y0,z0], [x1,y1,z1], ...])
        """
       
        T_list = self._allocate_time_3d(waypoints_3d)
        
        Q_all = get_global_Q(T_list, self.q_func)
        
        coeffs_3d = {
            'x': None,
            'y': None,
            'z': None
        }
        
        for i, axis in enumerate(['x', 'y', 'z']):
            axis_waypoints = waypoints_3d[:, i]
        
            A_eq, b_eq = build_constraints(axis_waypoints, T_list, n=self.n)
            
            if corridor_width is not None:
                A_ineq, l_ineq, u_ineq = self._build_corridor_constraints(
                    axis_waypoints, T_list, corridor_width
                )
                A_final = np.vstack([A_eq, A_ineq])
                l_final = np.hstack([b_eq, l_ineq])
                u_final = np.hstack([b_eq, u_ineq])
            else:
                A_final, l_final, u_final = A_eq, b_eq, b_eq
            
            coeffs_3d[axis] = solve_osqp(Q_all, A_final, l_final, u_final)
            
        return coeffs_3d, T_list

    def _allocate_time_3d(self, waypoints_3d):
        
        T_list = []
        d_acc_dec = (self.v_max**2) / (self.a_max)

        for i in range(len(waypoints_3d) - 1):
            dist = np.linalg.norm(waypoints_3d[i+1] - waypoints_3d[i])
            if dist > d_acc_dec:
            
                t_acc = self.v_max / self.a_max
                t_const = (dist - d_acc_dec) / self.v_max
                T = 2 * t_acc + t_const
            else:
                T = 2 * np.sqrt(dist / self.a_max)
            
            T_list.append(max(T, 0.2))
        
        return T_list


    def _build_corridor_constraints(self, axis_waypoints, T_list, width):
        
        A_eq = []
        l_eq = []
        u_eq = []
        M = len(T_list)
        num_params = M * (self.n + 1)
        for i in range(M):
           
            for fraction in [0.2, 0.4, 0.6, 0.8]:
                t_check = fraction * T_list[i]
                A_row = np.zeros(num_params)
                A_row[i*(self.n + 1) : (i + 1)*(self.n + 1)] = get_poly_basis(t_check, self.n, 0)
                
                center = axis_waypoints[i] + fraction * (axis_waypoints[i+1] - axis_waypoints[i])
               
                l = center - width
                u = center + width

                A_eq.append(A_row)
                l_eq.append(l)
                u_eq.append(u)
                
        return np.array(A_eq), np.array(l_eq), np.array(u_eq)


class TrajectoryVizNode(Node):
    def __init__(self):
        super().__init__('traj_planner_node')
        #create publisher,topic:'trajectory_viz'
        self.declare_parameter('frame_id', 'map')
        
        self.marker_pub = self.create_publisher(Marker, 'trajectory_viz',10)
        
        #实例化算法类
        self.planner = MinimumSnapPlanner3D()
        self.path = np.array([[0,0,0], [3,5,2], [6,0,4], [9,5,0]])
        self.corridor_w = 1.0
        self.all_coeffs, self.times = self.planner.plan(self.path, corridor_width=self.corridor_w)
        
        self.timer = self.create_timer(1.0,self.timer_callback)

        self.get_logger().info("node started!")

    def timer_callback(self):
        self.publish_path()
        self.publish_corridors()

    def publish_path(self):
        # 激活rviz可视化
        marker = self.init_marker("path", Marker.LINE_STRIP)
        marker.scale.x = 0.05  # 线宽
        marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0) # 青色轨迹

        n = self.planner.n

        # 对每段时间内分别采样
        for i in range(len(self.times)):
            # 在每段时间内，采样40个点
            t_samples = np.linspace(0, self.times[i], 40)

            for ts in t_samples:
                # 计算基向量
                basis = get_poly_basis(ts, n, 0)
                # 计算空间坐标，系数点乘基向量
                x = np.dot(self.all_coeffs['x'][i*(n+1) : (i+1)*(n+1)],basis)
                y = np.dot(self.all_coeffs['y'][i*(n+1) : (i+1)*(n+1)],basis)
                z = np.dot(self.all_coeffs['z'][i*(n+1) : (i+1)*(n+1)],basis)

                # 存放符合ros2 geometry_msgs/msg/消息结构的点
                p = Point()
                p.x, p.y, p.z = float(x), float(y), float(z)
                marker.points.append(p)

        self.marker_pub.publish(marker)

    def publish_corridors(self):
        marker = self.init_marker("corridors", Marker.CUBE_LIST)
        #调用init时，marker=0，要给通道另外整一个id，防止通道的更新覆盖到轨迹上去
        marker.id = 1

        # RViz 的 scale 是全长，corridor_width 是单侧宽度，所以乘 2
        marker.scale.x = marker.scale.y = marker.scale.z = self.corridor_w * 2
        marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.5) # 半透明黄

        # 我的飞行通道是在1/3处采样的，在1/3处取点，并把它们都连起来，用半透明粗线表示
        for i in range(len(self.path)-1):
            p_start = self.path[i]
            p_end = self.path[i+1]
            
            for f in [0.2, 0.4, 0.6, 0.8]:
                center = p_start + f * (p_end - p_start)

                p = Point()
                p.x, p.y, p.z = center.astype(float)
                marker.points.append(p)
                
        self.marker_pub.publish(marker)


    def init_marker(self, ns, marker_type):
        marker = Marker()
        marker.header.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = 0
        marker.type = marker_type
        marker.action = Marker.ADD
        #初始零旋转
        marker.pose.orientation.w = 1.0
        return marker

def main(args=None):
        rclpy.init(args=args)

        node = TrajectoryVizNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
        main()
        


