## 定义

任何一个 $n$ 阶的复数矩阵 $A$ 都与一个 Jordan 标准型近似。

考虑 $|\lambda I - A| = (\lambda - \lambda_1)^{e_1}(\lambda - \lambda_2)^{e_2}(...)(\lambda - \lambda_k)^{e_k}$，满足 $\sum_{i = 1}^{k} e_i = n$

那么，我们就可以写出 $A$ 对应的 Jordan 标准型为：

$$J = diag\{J_1, J_2, ..., J_k\}$$

其中，$J_j$ 是一个 Jordan 标准块，它的形式是：

$$J_j = \begin{bmatrix}\lambda_j & 1 & 0 & ... & 0\\ 0 & \lambda_j & 1 & ... & 0\\ 0 & 0 & \lambda_j & ... & 0\\ 0 & 0 & 0 & ... & 1\\ 0 & 0 & 0 & ... & \lambda_j\end{bmatrix}_{e_j \times e_j}$$

如果 $e_j = 1$，那么 Jordan 标准块就是一个常数。

举个例子：

$$\begin{bmatrix}1 & 1 & 0 & 0 & 0 & 0 & 0\\ 0 & 1 & 0 & 0 & 0 & 0 & 0\\ 0 & 0 & 2 & 0 & 0 & 0 & 0\\ 0 & 0 & 0 & 3 &  0 & 0 & 0\\ 0 & 0 & 0 & 0 & 4 & 1 & 0\\  0 & 0 & 0 & 0 & 0 & 4 & 1\\  0 & 0 & 0 & 0 & 0 & 0 & 4\end{bmatrix}$$

## 性质

对于同一个复矩阵 $A$，它对应的 Jordan 标准型在形式上不唯一（ Jordan 块的顺序可以改变），但本质上唯一（改变 Jordan 块的顺序得到的新 Jordan 标准型和原 Jordan 标准型相似）。
