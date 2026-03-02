# KKT 条件

本文摘自：[非线性优化中的 KKT 条件该如何理解？ - 王小龙的回答 - 知乎](https://www.zhihu.com/question/23311674/answer/235256926)

## 一、问题重述

KKT 条件的其中一种等价表述形式是：

$$\begin{aligned}\max_{x}\ \ \ &f(x)\\
s.t.\ \ \ h_j(x) &= 0,j=1,2,\cdots,q\\
g_i(x)&\le 0,i=1,2,\cdots,p
\end{aligned}$$

KKT 条件宣称，如果有一个 $x^{*}$ 是满足所有约束条件的极值点，则

$$\begin{aligned}\nabla f(x^*) &= \sum_j\lambda_j\nabla h_j(x^*)+\sum_i\mu_i\nabla g_i(x^*)\\
\lambda_j&\in\mathbb{R},\mu_i \ge 0,\mu_ig_i(x^*) = 0
\end{aligned}$$

简单说，就是在极值点处，$f$ 的梯度是一系列等式约束 $h_j$ 的梯度和不等式约束 $g_i$ 的梯度的线性组合。其中，等式约束梯度的权值 $\lambda_j$ 没有要求；不等式约束梯度的权值 $\mu_i$ 是非负的。并且，如果某个 $g_i(x^*)$ 严格小于 0 ，那么它的梯度不会出现在加权式中，因为此时 $\mu_i$ 必须等于 0 . 换句话说，只有 $x^*$ 恰好在边界 $g_i=0$ 上的那些 $g_i$ 的梯度才会出现在加权式中。

> 可以看出，KKT 条件可视为 Lagrange 乘子法的推广。

## 二、理解思路

对于一个优化问题，可行域是满足所有约束条件的 $n$ 维空间区域。对于可行域中的任意一点 $x$ ，朝着 $v$ 方向行走一点点，倘若还在可行域中，或者偏离可行域的量是行进距离的无穷小量，则称 $v$ 为 $x$ 的一个可行方向。我们用 $F(x)$ 表示 $x$ 的所有可行方向。

如果 $x^*$ 是一个满足所有约束条件的极值点，那么无论朝哪个可行方向走，$f$ 的值都不会增大。换句话说，在 $x^*$ ，让 $f$ 增大的方向不在 $F(x^*)$ 中。

这句话的精确表述，即为 KKT 条件。

## 三、几何思路

n 维空间中，$f(x) = 0$ 构成了一个曲面，专业术语叫流形。这个曲面是 n-1 维的，因为自变量的 n 个分量只满足一个方程。

!!! warning

    接下来，我们默认“曲面是光滑的”。

在曲面上的每一个点 $x$，都有一个梯度方向 $\nabla f(x)$，从而得到两个函数值增加/减少的方向：

$$\begin{aligned}
N_f^{+} &= \{\lambda \nabla f(x),\lambda \ge 0\}\\
N_f^{-} &= \{\lambda \nabla f(x),\lambda \le 0\}\\
N_f &= N_f^{+} \bigcup N_f^{-}
\end{aligned}$$

这里的 $N_f$ 是由梯度方向张成的一维子空间，过 $x$ 可以做一个与 $N_f$ 正交的 n-1 维切空间，即切平面 $T_f$ 。

从而，在曲面 $f(x) = 0$ 的一点 $x$ 周围，我们把空间分成了三部分：

$$N_f^{+}+N_f^{-}+T_f = \mathbb{R}^{n}$$

!!! note "子空间的“和”“直和”"

    子空间 $U,W$ 的和定义为 $U+W = \{u+w,u\in U,w\in W\}$ 。如果 $U,W$ 的交集是空集，那么称其为“直和”。

    值得注意的是，“和”与“并集”不是一个概念。举个例子，三维空间中，一条直线与一个不共面的平面，和是整个三维空间，但并集仅仅是该直线和平面上的所有点。

接下来，我们考察满足不同约束的点的可行方向。

对于等式约束 $h(x) = 0$ ，可行解 $x$ 必然在曲面上，也就是说它只能在曲面上移动。由于曲面是光滑的，所以它的可行方向是切平面方向，我们用 $T_h$ 表示。这是一个 n-1 维的切空间。

对于不等式约束 $g(x) < 0$ ，可行解 $x$ 必然在曲面外面，在曲面的其中一侧，它无论朝哪个方向行进一小步，都不会到曲面上来甚至穿越曲面。所以，它的可行方向是 $\mathbb{R}^n$ 。

对于不等式约束 $g(x) \le 0$ （考虑可行解 $x$ 在边界上的情况，我们称之为“起作用的不等式约束”），$x$ 要么在曲面上移动，要么往 $g < 0$ 的一侧跑，所以其可行方向是 $N_g^{-}+T_g$ 。这是一个 n 维的半空间。

总结：假设一个点满足约束，可行方向是这样一些方向，在这些方向上移动一点点，点仍然近似满足约束。对于等式约束，可行方向是切空间；对于不等式约束，点在边界上，可行方向是半空间，点不在边界上，可行方向是全空间。

显然地，对于多个不同约束，其可行方向是每个约束的可行方向的交集。

## 四、KKT 条件的证明

在上文我们提到，KKT 条件就是“在 $x^*$ ，让 $f$ 增大的方向不在 $F(x^*)$ 中”的严格表述。

我们知道，让 $f$ 增大的方向，就是与 $\nabla f(x^*)$ 成锐角的方向。因此，上面那句话可以用数学语言表述为

$$\forall u\in F(x^*),u\cdot \nabla f(x^*)\le 0$$

接下来，我们看看可行方向 $F(x^*)$ 中具体包含哪些方向 $u$ 。

对于等式约束 $h_j(x^*)=0$ ，$x^*$ 的可行方向是切平面方向 $T_{h_j} = \{u|u\cdot\nabla h_j(x^*) = 0, u\in \mathbb{R}^n\}$ 。

对于起作用的不等式约束 $g_i(x^*) \le 0$，可行方向是 $N_{g_i}^{-}+T_{g_i} = \{u|u\cdot\nabla g_i(x^*) \le 0, u\in\mathbb{R}^n\}$ 。

对于不起作用的不等式约束 $g_i(x^*) \le 0$，可行方向是 $\mathbb{R}^n$ 。

最终的可行方向集合是上述方向的交集，即：

$$\begin{aligned}
F(x^*) &= T_{h1}\bigcap T_{h_2}\bigcap\cdots\bigcap T_{h_q}\bigcap\left(N_{g_1}^{-}+T_{g_1}\right)\bigcap\left(N_{g_2}^{-}+T_{g_2}\right)\bigcap\cdots\bigcap\left(N_{g_p}^{-}+T_{g_p}\right)\bigcap\mathbb{R}^n\cdots\\
&= T_{h1}\bigcap T_{h_2}\bigcap\cdots\bigcap T_{h_q}\bigcap\left(N_{g_1}^{-}+T_{g_1}\right)\bigcap\left(N_{g_2}^{-}+T_{g_2}\right)\bigcap\cdots\bigcap\left(N_{g_p}^{-}+T_{g_p}\right)\\
&= \left\{u|\begin{aligned}u\cdot\nabla h_j(x^*)&=0,j=1,2,\cdots,q\\
u\cdot\nabla g_i(x^*) &\le 0,i =1,2,\cdots,p 且 g_i(x^*) = 0\\
u\in\mathbb{R}^n\end{aligned}\right\}
\end{aligned}$$

上文要求，$\forall u\in F(x^*),u\cdot \nabla f(x^*)\le 0$ ，不难看出下式是满足要求的一个形式。

$$\begin{aligned}\nabla f(x^*) &= \sum_j\lambda_j\nabla h_j(x^*)+\sum_{i\in active}\mu_i\nabla g_i(x^*)\\
\lambda_j&\in\mathbb{R},\mu_i \ge 0
\end{aligned}$$

Farkas 引理证明了该形式是唯一的，但我们这里就不再讨论了。

最后还有一个小问题，上式只包含了起作用的不等式约束 $g_i(x^*) = 0$，没有 $g_i(x^*)<0$ 。我们得想个办法，在数学形式上把不起作用的不等式约束也统一进来。

不妨规定：对于 $g_i(x^*) = 0$ ，权重 $\mu_i\ge0$ ；对于 $g_i(x^*)<0$ ，权重 $\mu_i=0$ 。这就巧妙形成了一个新条件：

$$\mu_ig_i(x^*) = 0$$

我们称之为“互补松弛条件”。最后，我们的结论是：

$$\begin{aligned}\nabla f(x^*) &= \sum_j\lambda_j\nabla h_j(x^*)+\sum_i\mu_i\nabla g_i(x^*)\\
\lambda_j&\in\mathbb{R},\mu_i \ge 0,\mu_ig_i(x^*) = 0
\end{aligned}$$

## 五、适用条件

KKT 条件成立有一个至关重要的前提：**约束品性（Constraint Qualification, CQ）**。它的存在，是为了保证极值点 $x^∗$ 附近的约束交集是一个形态正常的“光滑流形”。

最常见的 CQ 被称为**线性无关约束品性（LICQ）**：即在极值点 $x^∗$ 处，所有等式约束和起作用的不等式约束的梯度必须线性无关。换言之，由它们构成的雅可比矩阵必须满秩：

$$J = \begin{bmatrix}\nabla h_1(x^*)^\top\\
\vdots\\
\nabla h_q(x^*)^\top\\ \nabla g_1(x^*)^\top\\
\vdots\\
\nabla g_k(x^*)^\top
\end{bmatrix}$$

以三维空间中的两个曲面 $h_1=0$ 和 $g_2=0$ 为例，我们可以直观地看到满秩的意义：

如果 $\nabla h_1(x^∗)$ 与 $\nabla g_2(x^*)$ **线性无关（不共线）**，代表两曲面干脆利落地“横截相交”。在这种正常情况下，两个切平面的交线就是真实曲面交线的切线，代数方程算出的结果属于可行方向集合 $F(x^*)$

相反，如果它们**线性相关（共线）**，切平面重叠，说明两曲面相切了，此时交集退化成只有 $x^*$ 这唯一的孤立点。一旦如此，可行方向集合 $F(x^*)$ 就不复存在（为空集），代数推导出的方向成了空中楼阁，整个 KKT 条件的几何基石也就彻底崩塌了。

## 六、为什么直接用 KKT 条件求极值点？

让我们回顾上文逻辑：先假设极值点存在，然后推导极值点应该满足的条件，即 KKT 条件。由此看来，KKT 条件是极值点的必要条件，而非充分条件。为什么我们直接用 KKT 条件求极值呢？

### 凸优化问题

首先，对于凸优化问题，即目标函数是凸函数、可行域是一个凸集的函数，KKT 条件事实上是一个充要条件。我们就举本文最开始的例子：

$$\begin{aligned}\max_{x}\ \ \ &f(x)\\
s.t.\ \ \ h_j(x) &= 0,j=1,2,\cdots,q\\
g_i(x)&\le 0,i=1,2,\cdots,p
\end{aligned}$$

我们在这里要求 $f(x)$ 是上凸的（外国一般说是“凹的”，如果求最小值，就要求是“下凸的”“凸的”），$h_j(x)$ 是仿射函数（形如 $w^Tx+b$），$g_i(x)$ 是下凸的。

我们用数学语言表达。首先，因为 $f(x)$ 是上凸的，因此有

$$\begin{aligned}
f(\lambda x_1+(1-\lambda) x_2)&\ge \lambda f(x_1)+(1-\lambda)f(x_2)\\
f(\lambda(x_1-x_2)+x_2) &\ge \lambda [f(x_1)-f(x_2)]+f(x_2)\\
\frac{f(\lambda(x_1-x_2)+x_2)-f(x_2)}{\lambda} &\ge f(x_1)-f(x_2)
\end{aligned}$$

我们对两边取极限 $\lambda \to 0$ ，从而有

$$\nabla f(x_2)\cdot(x_1-x_2) \ge f(x_1)-f(x_2)$$

同理有

$$\begin{aligned}\nabla g_i(x_2)\cdot(x_1-x_2)&\le g_i(x_1)-g_i(x_2)\\
\nabla h_j(x_2)\cdot(x_1-x_2) &= h_j(x_1)-h_j(x_2)
\end{aligned}$$

我们接下来证明：对于凸优化问题，满足 KKT 条件的点 $x^*$ 就是极值点。即证明

$$\forall x \in Constraints, f(x^*) \ge f(x)$$

因为 $f(x)$ 是上凸的，我们有 $f(x^*) \ge f(x)+\nabla f(x^*) \cdot (x^* - x)$ 。代入 KKT 条件，推得：

$$\begin{aligned}
f(x^*) &\ge f(x)+\left[\sum_j\lambda_j\nabla h_j(x^*)+\sum_i\mu_i\nabla g_i(x^*)\right] \cdot (x^* - x)\\
\Rightarrow f(x^*)-f(x)&\ge \sum_j\lambda_j\nabla h_j(x^*)\cdot (x^* - x)+\sum_i\mu_i\nabla g_i(x^*)\cdot (x^* - x)
\end{aligned}$$

由于 $h_j(x)$ 是仿射函数，$g_i(x)$ 是下凸函数，进一步推得：

$$\begin{aligned}
f(x^*)-f(x)&\ge \sum_j\lambda_j[h_j(x^*)-h_j(x)]+\sum_i\mu_i[g_i(x^*)-g_i(x)]
\end{aligned}$$

其中，由于 $x$ 和 $x^*$ 都满足约束条件，有 $h_j(x^*)=h_j(x)=0$ ；根据互补松弛条件，有 $\mu_i g_i(x^*) = 0$ 。所以

$$\begin{aligned}
f(x^*)-f(x)\ge RHS &= \sum_j\lambda_j[0-0]+\sum_i[0-\mu_ig_i(x)]\\
&= -\sum\mu_ig_i(x) \ge 0\\
\Rightarrow f(x^*) &\ge f(x)
\end{aligned}$$

证毕！

### 非凸优化问题

此时情况变得相当复杂，理论做法是先用 KKT 条件筛选出一堆点，再用 Hessian Matrix 的正/负定性判断是否是局部极小/大值，再全局搜索。

但事实上，对于复杂的非凸优化问题，比如深度神经网络等等，上述方法的计算复杂度是无法接受的，现实中一般采用梯度下降、启发式搜索等方式，以期找到一个较优的局部极值点。
