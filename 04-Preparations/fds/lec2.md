# lec2 lists, stacks and queues
## ADT(abstract data type)
data type = {objects} ∪ {operations}
## list ADT
### array list
- objects:
$$\{item_0, item_1, \dots, item_{N-1}\}$$
- operations:
    - Finding_the_length：求链表长度 N
    - Printing：打印列表所有元素
    - Making_an_empty_list
    - Finding：查找第 k 项
    - Inserting：在第 k 项后插入新的项
    - Deleting：删除一项
    - Finding_next：查找下一个项
    - Finding_previous：查找上一个项，用于删除列表首项

- time complexity
    - 查找： $O(1)$
    - 删除或插入： $O(N)$ 由于array list占据连续的存储空间，该操作还会涉及大量数据移动

删除头：$O(N)$ （涉及到整个数组移动） 删除尾：$O(1)$

### linked list
占空间多，但不要求连续，查找慢，插入删除快。

虚拟头指针：数据为空，指针指向头节点。这样删除头节点就变方便了。

- time complexity
    - 查找：$O(N)$
    - 删除或插入：$O(1)$ （但是查找前一位指针需要时间，所以有了下面的双头链表）

### Double Linked Circular Lists
简化添加与删除的查找过程

### 多项式抽象数据类型 (The Polynomial ADT)

* **对象 (Objects)**:
$P(x) = a_1x^{e_1} + \dots + a_nx^{e_n}$，由一组有序对 $<e_i, a_i>$ 组成。
* $a_i$：**系数 (coefficient)**。
* $e_i$：**指数 (exponent)**，为非负数。


* **操作 (Operations)**:
* `Finding_degree`：判断多项式的最高次数。
* `Addition`：加法。
* `Subtraction`：减法。
* `Multiplication`：乘法。
* `Differentiation`：求导。

### Multilists

某种二维链表。


>To merge two singly linked ascending lists, both with N nodes, into one singly linked ascending list, the minimum possible number of comparisons is:  
最小比较次数：N（对于singly list单项链表，我所掌握的只有两个头节点，必须用链表B的头和链表A中从头到尾的所有元素依次比较）
最大的比较次数则是： 2N-1（最坏情况就是两个链表里的元素大小完全交替出现）

### Cursor Implementation游标实现
在指针实现中，Next 存的是内存地址（如 0x7ffee）；而在游标实现中，next 存的是数组的下标。
为了尽可能地利用空间，每次取用下标，不用后重新释放，通过free list标记哪些下标已占用哪些空闲，每次释放后下标放回free list开头

## stack ADT
- operations:
    - Int IsEmpty(Stack S);：检查栈是否为空
    - Stack CreateStack();：创建栈
    - DisposeStack(Stack S);
    - MakeEmpty(Stack S);：清空栈
    - Push(ElementType X, Stack S);：插入新元素
    - ElementType Top(Stack S);：**获得**栈顶元素
    - Pop(Stack S);：**删除**栈顶元素

>对空的栈使用 Pop 或 Top 操作将会引发栈 ADT 错误，对满的栈使用 Push 操作将会引发实现错误 (implementation error)

>在执行 Push 和 Pop 前必须进行错误检查


- 复杂度：Push, Pop, Top均发生在链表头部，所以复杂度 $O(1)$，但是多次调用`free()` `malloc()`开销大
- 改进方法：额外创建一个栈，用于存放本来应该 free() 掉的空间，等到有新的元素需要 push 的时候就可以用这个“回收站”中的空间，而无需再使用 malloc()

- 对于入栈序列 $1, 2, \dots, n$，一个出栈序列是合法的，当且仅当：对于序列中的任意元素 $a$，在它后面出来的、且比它小的所有元素，必须保持 **降序（从大到小）** 排列。

计算出栈顺序种类数：

**卡特兰数 (Catalan Number)**
若入栈序列长度为 $n$，则合法的出栈排列总数 $C_n$ 为：

$$C_n = \frac{1}{n+1} \binom{2n}{n} = \frac{(2n)!}{(n+1)!n!}$$

**前 5 项参考值：**
| $n$ (元素个数) | $C_n$ (合法排列数) |
| :--- | :--- |
| 1 | 1 |
| 2 | 2 |
| 3 | 5 |
| 4 | 14 |
| 5 | 42 |

**几何直观理解：网格路径法**
我们可以将出栈序列问题映射到二维坐标系中：
* **入栈**：向右走一步 $(+1, 0)$
* **出栈**：向上走一步 $(0, +1)$
* **约束条件**：路径从 $(0,0)$ 出发到达 $(n,n)$，且**不能越过 $y=x$ 这条对角线**（即任意时刻出栈次数不能超过入栈次数）。




### Balancing Symbols
思路简单，略，复杂度 $O(n)$ (在线算法，遍历一遍字符串就结束了)

### 逆波兰表达式 Postfix Evaluation
* 遇到操作数，将其压入栈中
* 遇到运算符 optopt，弹出栈最顶上两个元素 a,ba,b，其中 top=atop=a，然后计算c=b opt ac=b opt a，最后将 cc 压入栈中
* 遍历完后缀表达式后，栈中应当剩下一个元素，该元素即为最终结果
* 复杂度 $O(n)$ ：只遍历一次，但可能执行数次pop push等 ,由于栈的 Push 和 Pop 操作在链表或数组实现中都是 O(1) 的常数时间，且每个字符触发的栈操作次数是固定的（最多三次），因此处理每个字符的时间开销是恒定的。

### Infix to Postfix Conversion 中缀转后缀
- 规则 A：数字直接走
遇到数字（1, 2, 3...），直接写在后缀表达式里。
- 规则 B：符号看等级
    - 遇到运算符（+、-、*、/）：如果栈是空的，直接进去。
    - 如果你的等级 > 栈顶等级：你比他牛，你压在他上面（进栈）。
    - 如果你的等级 $\le$ 栈顶等级：栈顶出栈（输出），然后你再跟新的栈顶比，直到你比栈顶强或者栈空了，你再进去。
- 规则 C：括号是“结界”
  - 左括号 (：它是最霸道的。在外面时，它优先级最高，谁也挡不住它进栈；但一旦进栈，它就变弱了（优先级最低），为了让后面的符号都能压在它上面。
  - 右括号 )：它是“收割机”。一旦看到它，就把栈里的符号一个个弹出，直到看到对应的 ( 为止。最后把这一对括号丢掉（直接消除掉，不输出）。

## the Queue ADT
- Objects:  A finite ordered list with zero or more elements.
- Operations：
  - Int IsEmpty(Queue Q);：检查队列是否为空
  - Queue CreateQueue();：创建队列
  - DisposeQueue(Queue Q);
  - MakeEmpty(Queue Q);：清空队列
  - Enqueue(ElementType X, Queue Q);：入队 ⭐
  - ElementType Front(Queue Q);：获得队首元素 ⭐
  - Dequeue(Queue Q);：出队 ⭐
  
```c
struct  QueueRecord {
int     Capacity ;   /* max size of queue */
int     Front;          /* the front pointer */
int     Rear;           /* the rear pointer */
int     Size;  /* Optional - the current size of queue */
ElementType  *Array;    /* array for queue elements */
//ElementType 可以定义你希望存在队列里的元素类型
 } ; 
```

### circular queue
两种办法：
- 空出一块空间（如果没有留空，在队列空着和满着的时候，front都=rear）
- 增加一个 Size 的字段，用来实时统计队列元素个数，这样无需浪费空间（上述代码便采用这种做法）。如果用 front 表示队首元素，size 表示当前队伍大小，m 表示队伍最大大小，则队尾元素 rear = (front + size - 1) % m


## Trees
### definition
- 树 (trees)
- 根节点 (root)
- 子树 (subtrees) 
- 边 (edge)
- 度 (degree)
  - **一个节点的度**指的是它所有子树的个数
  - **一棵树的度**为 $$\max_{node \in tree} \{degree(node)\}$$
- 父节点 (parent)
- 孩子节点 (children)：父节点子树的根节点
- 兄弟节点 (siblings)：有共同父节点的孩子节点
- 叶子节点 (leaf)：度为 0 的节点
- 路径 (path)

- **顶点 $n_i$ 的深度 (depth)**：从根节点出发到 $n_i$ 的路径长度，规定 $depth(root) = 0$
- **顶点 $n_i$ 的高度 (height)**：从 $n_i$ 到叶子结点的**最长**路径长度，规定 $height(leaf) = 0$

$$\therefore height(root) = depth(deepest \ leaf)$$


- 路径长度 (length)：路径上边的条数
- 祖先 (ancestor)：从该节点到根节点的路径上所有的节点
- 后代 (descendant)：该节点所有子树的节点
- 内部节点 (internal vertices)：有孩子节点的顶点

### 树的公式

（0）对于任何一棵树（不一定是二叉树）：

* **边数与节点数的关系**：若树有 $N$ 个节点，则其边数（Edges）恒为 **$N - 1$**。
    
* **度（Degree）的关系**：所有节点的度之和等于边数。即：
    $$\sum \text{degree}(v) = N - 1$$
    *(注：在通用树中，节点的“度”通常指其子节点的个数。)*

（1）非空二叉树叶子结点数 = 度为 2 的结点数 + 1 即，$N_0 = N_2 + 1$

>例：There exists a binary tree with 2016 nodes in total, and with 16 nodes having only one child. （F）


（2）非空二叉树上第 $K$ 层至多有 $2^{K-1}$ 个结点（$K \ge 1$）

（3）高度为 $H$ 的二叉树至多有 $2^H - 1$ 个结点（$H \ge 1$）

（4）具有 $N$ 个（$N > 0$）结点的完全二叉树的高度为 $\lceil \log_2(N+1) \rceil$ 或 $\lfloor \log_2 N \rfloor + 1$

（5）对完全二叉树按从上到下、从左到右的顺序依次编号 $1, 2, \dots, N$，则有以下关系：

① 当 $i > 1$ 时，结点 $i$ 的双亲结点编号为 $\lfloor i/2 \rfloor$，即当 $i$ 为偶数时，其双亲结点的编号为 $i/2$，它是双亲结点的左孩子；当 $i$ 为奇数时，其双亲结点的编号为 $(i-1)/2$，它是双亲结点的右孩子。

② 当 $2i \le N$ 时，结点 $i$ 的左孩子编号为 $2i$，否则无左孩子。

③ 当 $2i + 1 \le N$ 时，结点 $i$ 的右孩子编号为 $2i + 1$，否则无右孩子。

④ 结点 $i$ 所在层次（深度）为 $\lfloor \log_2 i \rfloor + 1$。（设根结点为第 1 层）


(6)n叉树公式推广
这是关于 **$k$ 叉树（$k$-ary Tree）节点数量关系**的完整推导过程：


##### 1. 定义变量
在一个 $k$ 叉树中，一个节点最多可以有 $k$ 个孩子。令 $n_i$ 表示度为 $i$ 的节点数（即恰好有 $i$ 个孩子的节点数量）：
* $n_0$：叶子节点数。
* $n_1, n_2, \dots, n_k$：拥有不同数量孩子的分支节点数。
* $N$：总结点数。

##### 2. 建立方程组
**方程 I：节点总数恒等式**
总结点数等于所有度数节点之和：
$$N = n_0 + n_1 + n_2 + \dots + n_k$$

**方程 II：树的边数性质**
在任何树结构中，边数 $B$ 与节点数 $N$ 的关系始终为：
$$B = N - 1$$

**方程 III：边数与度的关系**
由于每条边都指向一个孩子节点，总边数等于所有节点指向的孩子总数：
$$B = 0 \cdot n_0 + 1 \cdot n_1 + 2 \cdot n_2 + \dots + k \cdot n_k$$

##### 3. 最终通用公式
$$n_0 = 1 + \sum_{i=2}^{k} (i-1)n_i$$




### 树的表达
- List Representation
每个节点的空间大小取决于它有多少个子树。
为了通用，你可能得按“最大可能的孩子数量”来定义结构体（比如每个节点都预留 10 个指针槽位）。如果大部分节点只有 1 个孩子，剩下的 9 个指针空间就全浪费了。

- FirstChild-NextSibling (左孩子右兄弟表示法)
    - FirstChild (长子)：指向它的第一个孩子。
    - NextSibling (亲兄弟)：指向它右边紧挨着的那个亲兄弟。
- 同一棵树表示不唯一（因为孩子的左右顺序不唯一）
  
### 二叉树 (binary tree)
#### 一般树与二叉树互转
图略……
补充：一般树（左图）的后序遍历 = 由上述方法得到的二叉树（右图）的中序遍历

####  表达式树

**1. 构造逻辑（由中缀表达式转换）：**
* **节点分类**：表达式树是一种特殊的二叉树。其**叶子节点**（Leaf Nodes）存储操作数（如常数或变量）；**内部节点**（Internal Nodes）存储运算符（如 `+`, `-`, `*`, `/`）。
* **层次结构**：构造时需遵循算术优先级。优先级较低的运算符（如最后执行的加减法）通常位于靠近**根部**的位置，而优先级较高的运算（或括号内的子表达式）则位于子树的深层。

**2. 遍历与表达式的关系：**
表达式树最核心的特性在于其遍历结果与三种表达式形式的一一对应关系：
* **中序遍历 (Inorder Traversal)**：得到**中缀表达式**（Infix Expression）。*注：还原时需根据子树结构手动添加必要的括号以保证优先级。*
* **后序遍历 (Postorder Traversal)**：得到**后缀表达式**（Postfix Expression，即逆波兰表达式）。
* **先序遍历 (Preorder Traversal)**：得到**前缀表达式**（Prefix Expression，即波兰表达式）。

#### 树的遍历
树的遍历 (tree traversals)：对树的每个节点都访问一次，时间复杂度为 $O(n)$

>知道前序或者后序遍历 + 中序遍历，可以确定唯一的一棵树
知道前序遍历 + 后序遍历，一般情况下无法确定树的形状

#####  Preorder Traversal 前序遍历

#####  Postorder Traversal 后序遍历 

#####  Levelorder Traversal 层序遍历

##### inorder traversal 中序遍历 

#### threaded binary tree

对于一般的二叉树，它的叶子节点的左右指针指向 NULL，这浪费了很多空间。而线索二叉树 (threaded binary trees)很好地利用了闲置的节点，具体规则如下：

- 如果 Tree->Left 为空，将它指向中序遍历中的前一个节点
- 如果 Tree->Right 为空，将它指向中序遍历中的后一个节点
- 有一个头节点(dummy node)，使得最左边和最右边孩子分别指向这个节点的左右孩子
>虽然这里默认使用中序遍历的定义，但我们也可以将其修改成前序或者后序遍历的版本 ( 比如对于后序遍历版的线索二叉树，某个节点空出来的左子树指向它在后序遍历中的前一个节点，空出来的右子树指向它在后序遍历中的后一个节点 )

```c
// 结构声明
typedef struct ThreadedTreeNode *PtrTo ThreadedNode;
typedef struct PtrToThreadedNode ThreadedTree;
struct ThreadedTreeNode
{
    int LeftThread;      // if it is True, then Left is a thread,not a child ptr
    TreadedTree Left;    
    ElementType Element;
    int RightThread;     // if it is True, then Right is a thread, not a child ptr
    ThreadedTree Right; 
}
```

- LeftThread (左标志位)
    - 如果 LeftThread == 0 (False): 表示 Left 是一个普通的指针，指向该节点的左孩子 (Left Child)。

    - 如果 LeftThread == 1 (True): 表示 Left 是一个线索 (Thread)，指向该节点在中序遍历下的前驱节点 (Predecessor)。

- RightThread (右标志位)
    - 如果 RightThread == 0 (False): 表示 Right 是一个普通的指针，指向该节点的右孩子 (Right Child)。

    - 如果 RightThread == 1 (True): 表示 Right 是一个线索 (Thread)，指向该节点在中序遍历下的后继节点 (Successor)。

#### special cases
* 歪斜二叉树 (skewed binary trees)：
退化为线性链表。
* 完全二叉树 (complete binary trees)：
除了最底层外，每一层都被填满，且最底层的结点都连续集中在左侧。

#### Binary Search Tree
每个节点有一个整数的键 (key)，每个键互不相同，非空左子树的键必须小于根上的键，非空右子树的键必须大于根上的键，左右子树也是二叉搜索树

左子树的**所有元素** < $R$ < 右子树的**所有元素**。


>对二叉搜索树的同一层从左往右遍历，得到的键的序列是有序的
通过对二叉搜索树的中序遍历得到的元素序列是有序的
给出一棵二叉搜索树的前序或者后序遍历，根据二叉搜索树的定义，我们应当可以还原出这棵树
对于一棵完全的二叉搜索树，它最小的节点一定是叶子节点，最大的就不一定了(这是完全二叉树从右往左的填充顺序决定的，最大的元素是“最右”的元素，他可能在导倒数第二排，且只有一个左孩子)

- operations
  -  SearchTree  MakeEmpty( SearchTree T ); 
  - Position  Find( ElementType X, SearchTree T ); 
  - Position  FindMin( SearchTree T ); 
  - Position  FindMax( SearchTree T ); 
  - SearchTree  Insert( ElementType X, SearchTree T ); 
  - SearchTree  Delete( ElementType X, SearchTree T ); 
  - ElementType  Retrieve( Position P ); 返回P值存储的数据
  
删除：删除度为 2的节点：用该节点左子树的最大节点或右子树的最小节点（挑一种）替换它自身，从子树中删除被替换的节点


## 补充
树的所有n节点的平均深度为Olog(n)
最坏情况：O(n)


前继(Predecessor)
后继(successor)


判定树 (Decision Tree)包含比较过程和比较结果分支