# Moore-Penrose 伪逆

## 1. 基本概念

Moore-Penrose 伪逆（Moore-Penrose pseudoinverse）是普通矩阵逆在**非方阵**或**奇异矩阵**情形下的推广。

通常记作：

$$A^\dagger.$$

若 $A$ 是可逆方阵，则：

$$\boxed{A^\dagger=A^{-1}}$$

---

## 2. Moore–Penrose 条件

对于任意矩阵 $A\in\mathbb R^{m\times n}$ ，其 Moore–Penrose 伪逆 $A^\dagger\in\mathbb R^{n\times m}$ 是唯一满足以下四个条件的矩阵：

$$\begin{aligned}
AA^\dagger A&=A\\
A^\dagger A A^\dagger&=A^\dagger\\
(AA^\dagger)^\top &=AA^\dagger\\
(A^\dagger A)^\top &=A^\dagger A
\end{aligned}
$$

---

## 3. 与最小二乘的关系

假设 $A\in\mathbb R^{m\times n}$ ，考虑线性方程：

$$
Ax=b.
$$
### 3.1 $b\notin\text{Col}(A)$

当 $b$ 不属于 $A$ 的列空间时，方程没有精确解。此时，考虑弱化问题为求解最小二乘问题：

$$\min_{x}\|Ax-b\|^2 .$$

定义目标函数 $f(x)$ 并展开，有：

$$\begin{aligned}
f(x)&=\|Ax-b\|^2\\
&=(Ax-b)^\top(Ax-b)\\
&=x^\top A^\top Ax-x^\top A^\top b-b^\top A x+b^\top b.
\end{aligned}$$

对 $x$ 求梯度，有：

$$\nabla_{x}f(x)=2A^\top Ax-2A^\top b.$$

最优点满足 $\nabla_x f(x) = 0$ ，即

$$\boxed{A^\top Ax=A^\top b}.$$

!!! Note "几何理解"

    设最小二乘解是 $x^*$ ，则 $Ax^*$ 是 $b$ 在 $A$ 的列空间上的正交投影。

    举个例子，$A_{3\times2}$ ，它的列空间是两个三维向量张成的平面，$b$ 这个三维向量不与该平面共面。此时，该平面上与 $b$ 最接近的向量，就是 $b$ 正交投影在该平面的向量 $Ax^*$ ，它们的差向量 $Ax^*-b$ 与该平面垂直。这意味着，$A$ 的每个列向量与 $Ax^*-b$ 的点乘结果都为 0 ，即

    $$A^\top(Ax^*-b)=0.$$

    因为 $A^\top AA^\dagger = A^\top$ ，所以满足以上方程的解为

    $$\boxed{x^*=A^\dagger b}.$$

    上式可通过 $AA^\dagger A=A$ 和 $(AA^\dagger)^\top =AA^\dagger$ 推出。

当 $A$ 不是列满秩时，$\text{Null}(A)\neq\{0\}$ ，因此方程的解可能不唯一。所有解可以表示为

$$\boxed{ x=A^\dagger b+(I-A^\dagger A)z, \qquad z\in\mathbb R^n }$$

其中

$$A(I-A^\dagger A) = A-AA^\dagger A = 0,$$

所以

$$(I-A^\dagger A)z\in\text{Null}(A).$$

即所有解都可以理解为特解+零空间中的任意向量。

### 3.2 $b\in\text{Col}(A)$

当 $b$ 属于 $A$ 的列空间时，方程存在精确解。我们设其中一个解为 $y$ ，即 $Ay=b$ 。

由 Moore-Penrose 条件

$$AA^\dagger A=A$$

可得

$$AA^\dagger b = AA^\dagger Ay = Ay = b.$$

因此

$$\boxed{x^*=A^\dagger b}$$

一定是 $Ax=b$ 的一个解。

同理，当 $A$ 不是列满秩时，其通解为：

$$\boxed{ x=A^\dagger b+(I-A^\dagger A)z, \qquad z\in\mathbb R^n }$$

又因为 $A^\dagger b$ 属于 $A$ 的行空间，而 $(I-A^\dagger A)z$ 属于 $A$ 的零空间，并且

$$\text{Row}(A)\perp\text{Null}(A),$$

!!! NOTE "$A^\dagger b\in \text{Row}(A)$"

    $$A^\dagger = A^\top(AA^\top)^\dagger.$$

    上式留待后文证明。由上式可得，

    $$\begin{aligned}
    A^\dagger b&=A^\top(AA^\top)^\dagger b\\
    &= A^\top\left[(AA^\top)^\dagger b\right]
    \end{aligned}$$

    注意到，$A^\top\left[(AA^\top)^\dagger b\right]$ 等价于 $A^\top y$ ，是 $A$ 的行向量的线性组合。因此 $A^\dagger b$ 属于 $A$ 的行空间。

!!! NOTE "行空间与零空间正交"

    $$\forall z\in\text{Null}(A),\quad Az=0.$$

    而 $Az=0$ 展开来看：

    $$\begin{bmatrix}a_1^\top z\\a_2^\top z\\\vdots\\a_m^\top z\end{bmatrix}=0.$$

    这说明 $z$ 与 $A$ 的每一个行向量都正交，也就说明行空间与零空间正交。

所以两部分相互正交。因此

$$\begin{aligned} \|x\|^2 &= \left\| A^\dagger b+(I-A^\dagger A)z \right\|^2\\ &= \|A^\dagger b\|^2 + \|(I-A^\dagger A)z\|^2\\ &\ge \|A^\dagger b\|^2. \end{aligned}$$

当零空间分量为 $0$ 时取等号，因此 $A^\dagger b$ 是所有精确解中二范数最小的解：

$$\boxed{ x^* = A^\dagger b = \arg\min_{Ax=b}\|x\|_2 }.$$

---

## 4. 一般情形：SVD

任意矩阵都可以进行奇异值分解（我们考虑常见的实数矩阵）：

$$
A=U\Sigma V^\top.
$$

其 Moore–Penrose 伪逆为：

$$
\boxed{
A^\dagger=V\Sigma^\dagger U^\top
}
$$

其中 $\Sigma^\dagger$ 的构造方法是：

$$\begin{aligned}
\sigma_i>0
\quad&\Longrightarrow\quad
\sigma_i^\dagger=\frac{1}{\sigma_i},\\
\sigma_i=0
\quad&\Longrightarrow\quad
\sigma_i^\dagger=0.
\end{aligned}$$

因此可以理解为：

> 在可逆的方向上取倒数，在不可逆的方向上保持为零。

!!! NOTE "证明 $A^\dagger = A^\top(AA^\top)^\dagger$"

    已知 $A^\dagger=V\Sigma^\dagger U^\top$ ，$AA^\top=U\Sigma V^\top V \Sigma^\top U^\top=U\Sigma\Sigma^\top U^\top$ ，则有

    $$(AA^\top)^\dagger = U(\Sigma\Sigma^\top)^\dagger U^\top.$$

    而 $A^\top=V\Sigma^\top U^\top$ ，因此 

    $$\begin{aligned}
    A^\top(AA^\top)^\dagger &= V\Sigma^\top U^\top U(\Sigma\Sigma^\top)^\dagger U^\top\\
    &= V\Sigma^\top (\Sigma\Sigma^\top)^\dagger U^\top.
    \end{aligned}$$

    接下来，我们只需要证明 $\Sigma^\dagger=\Sigma^\top(\Sigma\Sigma^\top)^\dagger$ 。我们考虑其中第 $i$ 行 $i$ 列的非零元素：

    $$\begin{aligned}
    \text{LHS}&=\frac{1}{\sigma_i}\\
    \text{RHS}&=\sigma_i\times\frac{1}{\sigma_i\times\sigma_i}=\frac{1}{\sigma_i}.\qquad\blacksquare
    \end{aligned}$$

---

## 5. 投影意义

有：

$$
\boxed{
AA^\dagger
=
P_{\operatorname{Col}(A)}
}
$$

即 $AA^\dagger$ 是到 $A$ 的列空间的正交投影。

!!! NOTE "说明"

    $AA^\dagger b$ 可以看成 $Ax$ ，是 $A$ 的列向量的线性组合，所以 $AA^\dagger$ 能把 $b$ 投影到 $A$ 的列空间。

    接下来，我们要说明 $AA^\dagger b$ 对 $b$ 的上述投影是正交投影，也即证明 $A$ 的任意列向量与 $AA^\dagger b-b$ 垂直，即证明：

    $$A^\top(AA^\dagger b-b) = 0.$$

    证明过程如下：

    $$\begin{aligned}
    b^\top(A-A) &=0\\
    b^\top(AA^\dagger A-A)&=0\\
    \left(A^\top(AA^\dagger)^\top-A^\top\right)b&=0\\
    (A^\top AA^\dagger-A^\top)b&=0\\
    A^\top(AA^\dagger b-b)&=0.
    \end{aligned}$$

类似地：

$$
\boxed{
A^\dagger A
=
P_{\operatorname{Row}(A)}
}
$$

即 $A^\dagger A$ 是到 $A$ 的行空间的正交投影。

> 证明不简单。

因此对于最小二乘问题：

$$
Ax^\star
=
AA^\dagger b
=
\operatorname{proj}_{\operatorname{Col}(A)} b.
$$

也就是说，当 $b\notin\operatorname{Col}(A)$ 时，伪逆先找到列空间中距离 $b$ 最近的点，再进行反解。

---

## 6. 常用公式总结

$$
\boxed{
A^\dagger=A^{-1}
}
\qquad
\text{（$A$ 为可逆方阵）}
$$

$$
\boxed{
A^\dagger=(A^\top A)^{-1}A^\top
}
\qquad
\text{（$A$ 满列秩）}
$$

$$
\boxed{
A^\dagger=A^\top (AA^\top)^{-1}
}
\qquad
\text{（$A$ 满行秩）}
$$

$$
\boxed{
A^\dagger=V\Sigma^\dagger U^\top
}
\qquad
\text{（一般情形）}
$$

以及：

$$
\boxed{
AA^\dagger=P_{\operatorname{Col}(A)},
\qquad
A^\dagger A=P_{\operatorname{Row}(A)}
}
$$
