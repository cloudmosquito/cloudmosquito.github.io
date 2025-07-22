## softmax 层

softmax 层接收一个实数向量 $\mathbf{z}=\begin{pmatrix}z_1 & z_2 & \cdots & z_k\end{pmatrix}$ ，并将其转变为一个元素非负且和为 1 的向量 $\mathbf{p}=\begin{pmatrix}p_1 & p_2 & \cdots & p_k\end{pmatrix}$ . 

原始形式：

$$Softmax(z_j) = \dfrac{e^{z_j}}{\sum_{i=1}^{N}e^{z_i}}$$

Softmax 层很适合放在神经网络最后，将多分类的输出值转换为范围在\[0, 1\] 和为 1 的概率分布。此时，Softmax 层输出的第 $i$ 个分量 $p_i$，就是样本属于第 $i$ 类的概率。

### 数值稳定性

指数函数的作用是拉开不同数值间的概率差距，让数值大的类别，概率显著大于数值小的类别。但也有缺点：容易出现数值溢出的情况。为了解决这一问题，一般使用如下形式：

$$Softmax(z_j) = \frac{e^{z_j-D}}{\sum_{i=1}^{N}e^{z_i-D}}, \ \ \ D=\max_{k}z_k$$

### 偏导数

分别考虑两种情况的偏导数： $\dfrac{\partial}{\partial z_j} Softmax(z_j)$ 和 $\dfrac{\partial}{\partial z_i}Softmax(z_i), i\neq j$ .

$$\begin{aligned}
\frac{\partial}{\partial z_j}Softmax(z_j) 
&= \frac{\partial}{\partial z_j}\frac{e^{z_j}}{\sum_{i=1}^{N}e^{z_i}}\\
&= \frac{\partial}{\partial z_j}\left(1- \frac{\sum_{i=1}^{N}e^{z_i}-e^{z_j}}{\sum_{i=1}^{N}e^{z_i}}\right)\\
&= -\left(\sum_{i=1}^Ne^{z_i}-e^{z_j}\right)\frac{\partial}{\partial z_j}\frac{1}{\sum_{i=1}^{N}e^{z_i}}\\
&= \left(\sum_{i=1}^Ne^{z_i}-e^{z_j}\right)\frac{e^{z_j}}{\left(\sum_{i=1}^{N}e^{z_i}\right)^2}\\
&= \frac{\sum_{i=1}^{N}e^{z_i}-e^{z_j}}{\sum_{i=1}^{N}e^{z_i}}\frac{e^{z_j}}{\sum_{i=1}^{N}e^{z_i}}\\
&= \left(1-Softmax(z_j)\right)\cdot Softmax(z_j)\\
\frac{\partial}{\partial z_i}Softmax(z_j)
&= \frac{\partial}{\partial z_i}\frac{e^{z_j}}{\sum_{k=1}^{N}e^{z_k}}\\
&= -e^{z_j}\frac{e^{z_i}}{\left(\sum_{k=1}^{N}e^{z_k}\right)^2}\\
&= -\frac{e^{z_j}}{\sum_{k=1}^{N}e^{z_k}} \cdot \frac{e^{z_i}}{\sum_{k=1}^{N}e^{z_k}}\\
&= -Softmax(z_j) \cdot Softmax(z_i)
\end{aligned}$$

## tanh 函数

tanh 函数接收一个浮点数，输出一个取值在 (-1, 1) 之间的浮点数，其定义为

$$\tanh(x)=\frac{e^x-e^{-x}}{e^x+e^{-x}}$$

它的作用是将任意实数映射到 (-1, 1) 之间，并且满足输出“零中心化”，即 $\tanh(0) = 0$ .

tanh 函数也很适合放到神经网络最后，再乘个系数，将神经网络的输出映射到特定区间。

事实上，神经网络中神经元的激活函数就是在干这件事。常见的激活函数有 tanh, sigmoid, ReLu, Leaky ReLu, ELU 等。

### 导数（梯度）

$$\begin{aligned}
\frac{\mathrm{d}}{\mathrm{d}x}\tanh(x) &= \frac{(e^x+e^{-x})(e^x+e^{-x})-(e^x-e^{-x})(e^x-e^{-x})}{(e^x+e^{-x})^2}\\
&= \frac{(e^x+e^{-x})^2-(e^x-e^{-x})^2}{(e^x+e^{-x})^2}\\
&=1 - \tanh^2(x)
\end{aligned}$$

当 $|x|$ 很大时， $\tanh^2(x)\approx 1$ ，梯度趋近于 0 ，会出现 **梯度消失（vanishing gradient）** 的问题。

在 $x$ 接近 0 附近，梯度最大，最利于调整信号。