# 哈密顿-凯莱定理（Hamilton-Cayley theorem）

!!! theorem "Hamilton-Cayley 定理"

    设 A 是数域 P 上的 n 阶矩阵， $f(λ)=|λE-A|=λ^n+b_1λ^{n-1}+…+b_{n-1}λ+b_n$ 是A的特征多项式，则 $f(A) = A^n + b_1A^{n-1}+...+b_{n-1}A+b_nI=0$ 。

## 证明

首先，设 $B(\lambda)$ 是 $\lambda I - A$ 的**伴随矩阵**。

!!! info "伴随矩阵"

    对于矩阵 $A = (a_{ij})_{n\times n}$ ，其伴随矩阵 $A^{*} = (A_{ij})_{n\times n}$ ，其中 $A_{ij} = (-1)^{i+j}M_{ij}$ ，$M_{ij}$ 是矩阵 $A$ 去除掉 $i$ 行 $j$ 列的行列式。

    伴随矩阵满足 $AA^{*} = |A|I$ .

$B(\lambda)$ 是一个 $\lambda$ 矩阵，可以表示为 $\lambda$ 的多项式。由于 $B(\lambda)$ 是 $\lambda I - A$ 的伴随矩阵，所以其元素中 $\lambda$ 的最高阶次是 $n-1$ 次，所以 $B(\lambda) = \lambda^{n-1}B_0 + \lambda^{n-2}B_1 + \cdots + B_{n-1}$ 。

!!! info "$\lambda$ 矩阵"

    $\lambda$ 矩阵指的是矩阵元素中包含变量 $\lambda$ 的矩阵，例如

    $$\begin{aligned}A(\lambda) &= \begin{bmatrix}\lambda^3 + \lambda^2 - 1 & \lambda^2-3\\ 2\lambda + 4 & 2\lambda^3+1\end{bmatrix}\\&=\lambda^3\begin{bmatrix}1 & 0\\ 0 & 1\end{bmatrix} + \lambda^2\begin{bmatrix}1 & 1\\ 0 & 0\end{bmatrix} + \lambda\begin{bmatrix}0 & 0\\2 & 0\end{bmatrix} + \begin{bmatrix}-1 & -3\\4 & 1\end{bmatrix}\end{aligned}$$

从而有

$$\begin{aligned}B(\lambda)(\lambda I - A) &= \left(\lambda^{n-1}B_0 + \lambda^{n-2}B_1 + \cdots + B_{n-1}\right) (\lambda I -A)\\
&= \lambda^nB_0 + \lambda^{n-1}(B_1-B_0A) + \cdots + \lambda(B_{n-1}-B_{n-2}A)-B_{n-1}A
\end{aligned} \tag{2-1}$$

设 $\lambda I -A = \lambda^n + a_1\lambda^{n-1} + a_2\lambda^{n-2} + \cdots + a_{n-1}\lambda + a_n$ ，则有

$$\begin{aligned}
B(\lambda)(\lambda I-A)&=(\lambda I-A)^{*}(\lambda I-A) \\
&= |\lambda I-A|I \\
&= \lambda^n I + a_1\lambda^{n-1} I + \cdots + a_{n-1}\lambda I + a_n
\end{aligned}\tag{2-2}$$

比较一下（2-1）式和（2-2）式，我们有

$$\begin{cases}B_0 &= I\\
B_1-B_0A &= a_1I\\
&\vdots\\
B_{n-1}-B_{n-2}A &= a_{n-1}I\\
-B_{n-1}A &=a_nI
\end{cases} \tag{2-3}$$

依次用 $A^n, A^{n-1}, \cdots, A, I$ 右乘（2-3）式，得到

$$\begin{cases}B_0A^n &= A^n\\
B_1A^{n-1}-B_0A^n &= a_1A^{n-1}\\
&\vdots\\
B_{n-1}A-B_{n-2}A^2 &= a_{n-1}A\\
-B_{n-1}A &=a_nI
\end{cases}\tag{2-4}$$

（2-4）式左右两边分别全部上下求和，左边求和结果为 0 ，右边求和结果为 $f(A)$ ，故 $f(A) = 0$ 得证。

## 意义

该定理为我们提供这样一个推论：

!!! theorem

    矩阵 $A^{k}, k \ge n$ 总可以用矩阵 $A^{j}, j=0,1,2,\cdots,n-1$ 的线性组合表示。

在控制领域，我们经常遇到矩阵 $e^{At}$ ，此时可如下使用该推论：

$$\begin{aligned}e^{At} &= I+At + \frac{1}{2!}(At)^2 + \frac{1}{3!}(At)^3 + \cdots\\
&= \beta_0(t)I + \beta_1(t)A + \beta_2(t)A^2 + \cdots \beta_{n-1}(t)A^{n-1}
\end{aligned}$$
