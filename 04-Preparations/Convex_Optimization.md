# 最优化、凸优化

## 一、 基本概念
我看的是[这个](https://www.bilibili.com/video/BV19M4y1n7M3/?share_source=copy_web&vd_source=6a65513384955cc33f848d6a6894e1a1)，感觉勉强够用了

### 1. 最优化问题的一般形式
$$
\begin{cases} 
\min f(x) & \text{(目标函数)} \\
x \in C & \text{(约束条件/满足条件)} 
\end{cases}
$$

### 2. 凸集 (Convex Set)
* **定义**：假设集合 $X \subset \mathbb{R}^n$，如果对任意两点 $x_1, x_2 \in X$ 和 $\alpha \in (0, 1)$，总有：
    $$\alpha x_1 + (1-\alpha) x_2 \in X$$
    则称 $X$ 是凸集。
* **直观理解**：集合内任意两点连线上的每一点均处于该集合内。

### 3. 凸函数与凹函数
* **凸函数 (Convex Function)**：
    * **定义**：假设集合 $X \subset \mathbb{R}^n$ 且 $f: X \to \mathbb{R}$。若对于任意 $x_1, x_2 \in X$ 和 $\alpha \in (0, 1)$，有：
        $$f[\alpha x_1 + (1-\alpha) x_2] \le \alpha f(x_1) + (1-\alpha) f(x_2)$$
    * **严格凸函数**：当上式不等号取严格小于号（$<$）时。
    * **几何特征**：开口向上 ($\cup$)。
* **凹函数 (Concave Function)**：
    * **定义**：满足 $f[\alpha x_1 + (1-\alpha) x_2] \ge \alpha f(x_1) + (1-\alpha) f(x_2)$。
    * **几何特征**：开口向下 ($\cap$)。类似于凸函数取负数。
* **非凸非凹函数**：如 $\sim$ 型曲线，既包含凸区间也包含凹区间。

---

## 二、 凸优化定义及其性质

### 1. 定义
$$
\begin{cases} 
\min f(x) \\
\text{s.t. } x \in C
\end{cases}
$$
满足以下两个条件：
1.  **$C$ 是凸集**。
2.  **目标函数 $f(x)$ 是 $C$ 上的凸函数**。

> **核心原则**：凸优化的性质极好，在建模时应尽量将问题转化为凸优化问题。

### 2. 性质
1.  **局部极小点就是全局极小点**。
2.  若目标函数是**严格凸函数**，则凸优化问题具有**唯一的全局极小点**。
3.  凸优化的**全局极大点**（若存在）必能在可行域的**边界**上达到。

---

## 三、 导数工具：梯度、Jacobi 与 Hessian

### 1. 梯度 (Gradient)
* **适用**：$f: \mathbb{R}^n \to \mathbb{R}$（多输入，单输出的一阶导数）。
* **定义**：若 $f$ 关于各分量的一阶偏导数均存在，则其梯度向量为：
    $$\nabla f(x) = \left[ \frac{\partial f(x)}{\partial x_1}, \frac{\partial f(x)}{\partial x_2}, \dots, \frac{\partial f(x)}{\partial x_n} \right]^T$$

### 2. Jacobi 矩阵 (Jacobian Matrix)
* **适用**：$f: \mathbb{R}^n \to \mathbb{R}^m$（$n$ 维输入，$m$ 维输出函数的一阶导数）。
* **定义**：设 $f = (f_1, f_2, \dots, f_m)$，其 Jacobi 矩阵为：
    $$Df(x) = \begin{bmatrix} 
    \frac{\partial f_1(x)}{\partial x_1} & \frac{\partial f_1(x)}{\partial x_2} & \dots & \frac{\partial f_1(x)}{\partial x_n} \\
    \frac{\partial f_2(x)}{\partial x_1} & \frac{\partial f_2(x)}{\partial x_2} & \dots & \frac{\partial f_2(x)}{\partial x_n} \\
    \vdots & \vdots & \ddots & \vdots \\
    \frac{\partial f_m(x)}{\partial x_1} & \frac{\partial f_m(x)}{\partial x_2} & \dots & \frac{\partial f_m(x)}{\partial x_n}
    \end{bmatrix}_{m \times n}$$

### 3. Hessian 矩阵
* **适用**：$f: \mathbb{R}^n \to \mathbb{R}$（多输入，单输出的二阶导数）。
* **定义**：梯度向量 $\nabla f$ 的 Jacobi 矩阵即为 Hessian 矩阵。若二阶偏导均存在：
    $$\nabla^2 f(x) = \begin{bmatrix} 
    \frac{\partial^2 f}{\partial x_1^2} & \frac{\partial^2 f}{\partial x_1 \partial x_2} & \dots & \frac{\partial^2 f}{\partial x_1 \partial x_n} \\
    \frac{\partial^2 f}{\partial x_2 \partial x_1} & \frac{\partial^2 f}{\partial x_2^2} & \dots & \frac{\partial^2 f}{\partial x_2 \partial x_n} \\
    \vdots & \vdots & \ddots & \vdots \\
    \frac{\partial^2 f}{\partial x_n \partial x_1} & \frac{\partial^2 f}{\partial x_n \partial x_2} & \dots & \frac{\partial^2 f}{\partial x_n^2}
    \end{bmatrix}_{n \times n}$$