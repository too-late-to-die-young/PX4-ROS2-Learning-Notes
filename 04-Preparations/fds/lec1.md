reference：
https://note.tonycrane.cc/cs/algorithm/ds/topic1/
https://note.howjul.com/FDS/fds_lec/fds_lec1-6/
https://note.noughtq.top/algo/fds/2

# lec1
## 1. algorithm
- input: can be zero
- output: one or more
- definiteness
- finiteness: finite number of steps
- effectiveness

>a program does not have to be finite


## 2. analyze

### what & how

- what to analyze
    - run time 与机器和编译器有关
    - time & space complexity 与机器和编译器无关

- assumption
    - 指令按顺序执行
    - 所有指令（运算）都消耗同一时间单元
    - 数据规模是给定的，且有无限空间

- 一般需要分析 $T_{avg}(N)$（平均情况）和 $T_{worst}(N)$（最差情况），$N$ 是输入的数据规模（也可以有多个输入规模）。


### asymptotic notation
#### definition
##### 上界（worst case）：
$$T(N) = O(f(N))$$
if there are positive constants $c$ and $n_0$ such that $T(N) \leq c \cdot f(N)$ for all $N \geq n_0$.

##### 下界：
$$T(N) = \Omega(g(N))$$
if there are positive constants $c$ and $n_0$ such that $T(N) \geq c \cdot g(N)$ for all $N \geq n_0$.

##### 上下界相同：
$$T(N) = \Theta(h(N))$$
if and only if $T(N) = O(h(N))$ and $T(N) = \Omega(h(N))$.

##### 复杂度严格小于上界：
$$T(N) = o(p(N))$$
if $T(N) = O(p(N))$ and $T(N) \neq \Theta(p(N))$.


#### rules

- If $T_1(N) = O(f(N))$ and $T_2(N) = O(g(N))$, then
  - (a) $T_1(N) + T_2(N) = \max(O(f(N)), O(g(N)))$
  - (b) $T_1(N) * T_2(N) = O(f(N) * g(N))$

- If $T(N)$ is a polynomial of degree $k$, then $T(N) = \Theta(N^k)$.

- $\log^k N = O(N)$ for any constant $k$. This tells us that **logarithms grow very slowly**.

- for input n (复杂度递增):
https://www.bigocheatsheet.com/

    | Time      | Name        |
    |-----------|-------------|
    | $1$       | constant    |
    | $\log n$  | logarithmic |
    | $n$       | linear      |
    | $n \log n$| log linear  |
    | $n^2$     | quadratic   |
    | $n^3$     | cubic       |
    | $2^n$     | exponential |
    | $n!$      | factorial   |


### how to calculate time complexity
**算法复杂度分析** 

---

## (I) 一层循环：对数阶分析

### **解题思路：**

1. **列出**循环趟数 $t$ 及每轮循环变量（如 $i$）的变化值
2. **找到** $t$ 与 $i$ 的数学关系
3. **确定**循环停止条件
4. **联立**两式，解方程求出 $t$
5. **写出**最终结果

### **例题**

```c
i = n * n;
while ( i != 1 )
    i = i / 2;

```

* **推导过程：**
* $t$ (次数): $0, 1, 2, 3, \dots, t$
* $i$ (变量): $n^2, \frac{n^2}{2}, \frac{n^2}{4}, \frac{n^2}{8}, \dots, \frac{n^2}{2^t}$


* **寻找关系：** $i = \frac{n^2}{2^t}$ —— ①
* **停止条件：** $i = 1$ —— ②
* **联立解方程：** $\frac{n^2}{2^t} = 1 \implies n^2 = 2^t \implies t = \log_2(n^2)$
* **结论：** $T(N) = O(\log_2 n)$

---

## (II) 两层循环：求和法

### **解题思路：**

1. **列出**外层循环中核心变量的变化值
2. **列出**在对应变量下，内层语句执行的次数
3. **求和**并写出结果

### **[例题 1]**

```c
int m = 0, i, j;
for (i = 1; i <= n; i++)
    for (j = 1; j <= 2 * i; j++)
        m++;

```

* **推导过程：**
* 当 $i = 1, 2, 3, \dots, n$ 时，内层分别执行 $2, 4, 6, \dots, 2n$ 次。


* **等差数列求和：** $\frac{n(2 + 2n)}{2} = n(n + 1)$
* **结论：** $T(N) = O(n^2)$

### **例题**

```c
for ( i = n - 1; i > 1; i-- )
    for ( j = 1; j < i; j++ )
        if ( A[j] > A[j+1] )
            A[j]与A[j+1]交换;

```

* **分析：** 外层 $i$ 从 $n-1$ 减至 $2$，内层执行次数随 $i$ 递减，依然构成等差数列，复杂度为 **$O(n^2)$**。

---

## (III) 多层循环：三维抽象与列式求和

### **解题思路：**

* **方法一：** 抽象为计算三维体积
* **方法二：** 嵌套列式求和

### **例题**

```c
for (i = 0; i <= n; i++)
    for (j = 0; j <= i; j++)
        for (k = 0; k < j; k++)
            S++;

```

* **方法二推导（列式求和）：**
1. 最内层执行次数：$\sum_{k=0}^{j-1} 1 = (j - 1 - 0 + 1) = j$
2. 中层求和：$\sum_{j=0}^{i} j = \frac{i(i+1)}{2}$
3. 外层总计：$\sum_{i=0}^{n} \frac{i(i+1)}{2} = \frac{1}{2} \sum_{i=0}^{n} (i^2 + i) = \frac{1}{2} \sum i^2 + \frac{1}{2} \sum i$

* **结论：** $T(N) = O(n^3)$

---
## （IV）递归复杂度分析
### 迭代展开法
**例：**

---

**1. 基础迭代展开：**


$$T(N) = 2T\left(\frac{N}{2}\right) + cN$$

$$T\left(\frac{N}{2}\right) = 2T\left(\frac{N}{2^2}\right) + c\frac{N}{2}$$

$$T\left(\frac{N}{4}\right) = 2T\left(\frac{N}{2^3}\right) + c\frac{N}{2^2}$$

**2. 将子项代入原式：**


$$T(N) = 2 \left( 2T\left(\frac{N}{2^2}\right) + c\frac{N}{2} \right) + cN$$

$$= 2^2 T\left(\frac{N}{2^2}\right) + 2 \cdot cN$$

**3. 推广至第 $k$ 层迭代的一般形式：**


$$= 2^k \cdot T\left(\frac{N}{2^k}\right) + c \cdot k N$$

**4. 设定终止条件：**
令 $2^k = N$，由此推导出迭代深度 $k = \log_2 N$。
将 $k$ 代入一般形式：


$$= N \cdot T(1) + c \cdot N \cdot \log_2 N$$

---

**最终结论：**
该算法的时间复杂度为 $O(N \log N)$。

### 主定理
主定理用于快速求解形如 $T(n) = aT(n/b) + f(n)$ 的递归方程时间复杂度，其中 $a \geq 1, b > 1$。

根据 $f(n)$ 与 $n^{\log_b a}$ 的量级比较，分为以下三种情况：

* **情况 1：**
如果存在常数 $\epsilon > 0$ 使得 $f(n) = O(n^{\log_b a - \epsilon})$，则：

$$T(n) = \Theta(n^{\log_b a})$$


* **情况 2：**
如果 $f(n) = \Theta(n^{\log_b a})$，则：

$$T(n) = \Theta(n^{\log_b a} \log n)$$


* **情况 3：**
如果存在常数 $\epsilon > 0$ 使得 $f(n) = \Omega(n^{\log_b a + \epsilon})$，且同时存在常数 $c < 1$ 使得对于充分大的 $n$ 有 $af(n/b) \leq cf(n)$，则：

$$T(N) = \Theta(f(n))$$


## 3. compare the algorithm

### 例：寻找和最大子序列


#### divide and conquer（分治算法）
- 对于算法的解释：当你把数组从中间一分为二时，那个“最大的子序列”只可能出现在三个地方：全都在左半部分；全都在右半部分；跨越了中间点，左右各占一部分。
- 使用递归迭代，不断寻找和最大的序列是位于左边、右边还是中间。

复杂度计算见上面迭代的例子

#### 在线（on-line）算法
“负值归零（Resetting）：else if (ThisSum < 0) ThisSum = 0;
如果当前的子序列和 ThisSum 已经变成了负数，那么它对后续任何序列的贡献都只会是“减法”。
与其带着这个“负资产”继续往后加，不如直接将其归零，从下一个元素开始重新计算子序列。”

复杂度o（n）
不回头，只读一遍，在任意时刻停下来，得到的都是已经读入数据中的解

### 例：三种复杂度为 $\log N$ 的算法！
#### Binary Search
```c
int BinarySearch(const ElementType A[ ], ElementType X, int N)
{
    int Low, Mid, High;

    Low = 0;
    High = N - 1;
    while (Low <= High)
    {
        Mid = (Low + High) / 2;
        if (A[Mid] < X)
            Low = Mid + 1;
        else if (A[Mid] > X)
            High = Mid - 1;
        else
            return Mid;
    }
    return NotFound;  // NotFound被定义为-1
}
```

#### Euclid’s Algorithm 辗转相除法求最大公约数
关键在于证明：若 $M > N$，则 $M \bmod N < M/2$。

#### Exponentiation 快速幂运算
```c
long int Pow(long int X, unsigned int N) {
    if (N == 0) return 1;
    if (N == 1) return X;
    
    if (N % 2 == 0)
        return Pow(X * X, N / 2); // 偶数情况
    else
        return Pow(X * X, N / 2) * X; // 奇数情况
}
```

* **如果 $N$ 是偶数：**

$$X^N = X^{N/2} \cdot X^{N/2} = (X^{N/2})^2$$


* **如果 $N$ 是奇数：**

$$X^N = X \cdot X^{N-1} = X \cdot (X^{(N-1)/2})^2$$



## 4. 复杂度自查

- 若 $\frac{T(2N)}{T(N)} \approx 2$，则算法可能是 $O(N)$（线性增长）。

- 若 $\frac{T(2N)}{T(N)} \approx 4$，则算法可能是 $O(N^2)$（平方增长）。


当 $T(N) = O(f(N))$ 时，

根据大 $O$ 记号的数学定义，当 $N \to \infty$ 时：

$$\lim_{N \to \infty} \frac{T(N)}{f(N)} = C$$

其中 $C$ 是一个**非负常数**（$0 \le C < \infty$）。


