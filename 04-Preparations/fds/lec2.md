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


### Balancing Symbols
思路简单，略，复杂度 $O(n)$ (在线算法，遍历一遍字符串就结束了)

### 逆波兰表达式 Postfix Evaluation
* 遇到操作数，将其压入栈中
* 遇到运算符 optopt，弹出栈最顶上两个元素 a,ba,b，其中 top=atop=a，然后计算c=b opt ac=b opt a，最后将 cc 压入栈中
* 遍历完后缀表达式后，栈中应当剩下一个元素，该元素即为最终结果
* 复杂度 $O(n)$ ：只遍历一次，但可能执行数次pop push等 ,由于栈的 Push 和 Pop 操作在链表或数组实现中都是 O(1) 的常数时间，且每个字符触发的栈操作次数是固定的（最多三次），因此处理每个字符的时间开销是恒定的。

### Infix to Postfix Conversion 中缀转后缀
