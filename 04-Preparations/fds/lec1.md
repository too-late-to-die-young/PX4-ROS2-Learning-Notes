reference：
https://note.tonycrane.cc/cs/algorithm/ds/topic1/
https://note.howjul.com/FDS/fds_lec/fds_lec1-6/

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
$$T(N) = O(f(N))$$ if there are positive constants $c$ and $n_0$ such that $T(N) \leq c \cdot f(N)$ for all $N \geq n_0$.

##### 下界：
$$T(N) = \Omega(g(N))$$ if there are positive constants $c$ and $n_0$ such that $T(N) \geq c \cdot g(N)$ for all $N \geq n_0$.


##### 上下界相同：
$$T(N) = \Theta(h(N))$$ if and only if $T(N) = O(h(N))$ and $T(N) = \Omega(h(N))$.

##### 复杂度无限接近上界：
$$T(N) = o(p(N))$$ if $T(N) = O(p(N))$ and $T(N) \neq \Theta(p(N))$.

#### rules

- If $T_1(N) = O(f(N))$ and $T_2(N) = O(g(N))$, then
  - (a) $T_1(N) + T_2(N) = \max(O(f(N)), O(g(N)))$
  - (b) $T_1(N) * T_2(N) = O(f(N) * g(N))$

- If $T(N)$ is a polynomial of degree $k$, then $T(N) = \Theta(N^k)$.

- $\log^k N = O(N)$ for any constant $k$. This tells us that **logarithms grow very slowly**.

- for input n (复杂度递增):

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








