# LQR

## 概述

LQR，全称 Linear Quadratic Regulator，即线性二次型调节器，是一种状态反馈控制器。

LQR 问题可以表述为

$$\begin{aligned}\min_{\mathbf{x}_{1:N}, \mathbf{u}_{1:N-1}}\quad &J=\frac{1}{2}\sum_{k=1}^{N-1}\left(\mathbf{x}_k^{\top}Q_k\mathbf{x}_k+\mathbf{u}_k^{\top}R_k\mathbf{u}_k\right)+\frac{1}{2}\mathbf{x}_N^{\top}Q_N\mathbf{x}_N\\
\mathrm{s.t.}\quad &\mathbf{x}_{k+1}=A_k\mathbf{x}_k+B_k\mathbf{u}_k
\end{aligned}$$

式中 $\mathbf{x} \in \mathbb{R}^{n}, \mathbf{u} \in \mathbb{R}^{m}, Q,A \in \mathbb{R}^{n\times n}, R \in \mathbb{R}^{m\times m}, B\in \mathbb{R}^{n\times m}$ . 系统在 $k$ 时刻的状态为 $\mathbf{x}_k$ ，此时施加控制 $\mathbf{u}_k$ ，下一时刻系统状态转变为 $\mathbf{x}_{k+1}$ .

用文字表述：对于一个具有线性状态方程的系统，构造如上确定性最优控制问题，其中代价函数是一个二次型，包括状态代价、阶段代价和终端代价。我们的目的是让系统状态最终为 $0$ （调节器）。通过解上述问题，我们获得一个时间窗内的最优控制序列，从而施加控制。

为了使上述问题有唯一解，我们要求：

$$Q \succeq 0,\quad R\succ 0$$

一般而言，我们会将 $Q$ 和 $R$ 都取成对角阵。

调参经验：

1. Bryson's rule：将 $Q$ 和 $R$ 都取成对角阵，对应项设为能接受的最大误差平方的倒数。这相当于做了一次归一化，使得代价函数中的每一项都有近似相等的地位，是很好的调参起点。
2. 若 $Q$ 和 $R$ 比值不变，共同增大或减小不改变最终的控制效果。

## 间接打靶法 Indirect Shooting

我们利用[庞特里亚金原理](./庞特里亚金原理.md)解 LQR 问题，有：

$$\begin{aligned}
H(\mathbf{x}_k,\mathbf{u}_k,\lambda_{k+1}) &= \frac{1}{2}\left(\mathbf{x}_k^{\top}Q_k\mathbf{x}_k+\mathbf{u}_k^{\top}R_k\mathbf{u}_k\right)+\lambda_{k+1}^{\top}(A_k\mathbf{x}_k+B_k\mathbf{u}_k)\\
l_{F}(\mathbf{x}_N) &= \frac{1}{2}\mathbf{x}_N^{\top}Q_N\mathbf{x}_N
\end{aligned}$$

从而得到：

$$\begin{aligned}
\mathbf{x}_{k+1}&=\nabla_{\lambda}H(\mathbf{x}_k,\mathbf{u}_k,\lambda_{k+1}) = A_k\mathbf{x}_k+B_k\mathbf{u}_k\\
\lambda_k &= \nabla_{\mathbf{x}}H(\mathbf{x}_k,\mathbf{u}_k,\lambda_{k+1}) = \frac{1}{2}\left(Q_k+Q_k^{\top}\right)\mathbf{x}_k + A_k^{\top}\lambda_{k+1}\\
\lambda_N &= \nabla_{\mathbf{x}}l_F(\mathbf{x}_N) = \frac{1}{2}\left(Q_{N}+Q_{N}^{\top}\right)\mathbf{x}_N\\
\mathbf{u}_k &= \arg\min_{\mathbf{u}}H(\mathbf{x}_k, \mathbf{u}_k, \lambda_{k+1})
\end{aligned}$$

由于 LQR 问题形式无其他约束（如果要考虑约束，就没有这么漂亮的解了），因此 $\mathbf{u}_k$ 即满足 $\dfrac{\partial H}{\partial \mathbf{u}} = \mathbf{0}$ 的点：

$$\begin{aligned}
\frac{\partial H}{\partial \mathbf{u}} &= \frac{1}{2}\mathbf{u}_k^{\top}\left(R_k+R_k^{\top}\right) + \lambda_{k+1}^{\top}B_k = \mathbf{0}\\
\implies\mathbf{u}_k &= -2\left(R_k^{\top}+R_k\right)^{-1}B_k\lambda^{\top}_{k+1}
\end{aligned}$$

一般情况下， $Q$ 和 $R$ 都是对称矩阵，从而有：

$$\begin{aligned}
\mathbf{x}_{k+1}&= A_k\mathbf{x}_k+B_k\mathbf{u}_k\\
\lambda_k &= Q_k\mathbf{x}_k + A_k^{\top}\lambda_{k+1}\\
\lambda_N&=Q_N\mathbf{x}_N\\
\mathbf{u}_k &= -R_k^{-1}B_k\lambda_{k+1}^{\top}
\end{aligned}$$

间接打靶法的流程如下：

1. 估计 $\mathbf{u}_{1:N}$ ；
2. 根据状态初值 $\mathbf{x}_1$ 和估计控制序列 $\mathbf{u}_{1:N}$ ，利用状态方程模拟得到 $\mathbf{x}_{1:N}$ ；
3. 先用 $\mathbf{x}_{N}$ 计算 $\lambda_N$ ，然后根据 $\lambda_{k+1}$ 计算 $\lambda_k$ 和理想 $\mathbf{u}_{k}$ ，从而得到一组理想控制序列 $\mathbf{u}^{*}_{1:N}$ ，再计算 $\Delta \mathbf{u}_{1:N} = \mathbf{u}^{*}_{1:N} - \mathbf{u}_{1:N}$ ；
4. 利用 $\Delta\mathbf{u}_{1:N}$ 更新 $\mathbf{u}_{1:N}$ ，更新时以代价函数 $J$ 为评价函数（此处会计算新的一组 $\mathbf{x}_{1:N}$）执行回溯线搜索，确定实际更新步长；
5. 若 $\Delta \mathbf{u}$ 小于阈值，则认为控制序列已收敛，程序停止，反之，跳到第三步继续执行。

间接打靶法不是一个很好的方法，因为其计算效率太低了，尤其是对于时间窗口较长的情况。

## 视作二次规划

通过适当的换元，我们可以将 LQR 问题转化为一个 QP 问题，从而使用[优化理论速成](./优化理论速成.md)中的内容求解。

首先，需要注意到，系统的初始状态 $\mathbf{x}_1$ 是不可改变、不可优化的，因此，代价函数中的 $\frac{1}{2}\mathbf{x}_1^{\top}Q_1\mathbf{x}_1$ 一项可以舍去。

定义

$$\begin{aligned}\mathbf{z} &= \begin{bmatrix}\mathbf{u}_1\\\mathbf{x}_2\\\mathbf{u}_2\\\mathbf{x}_3\\\vdots\\\mathbf{u}_{N-1}\\\mathbf{x}_N\end{bmatrix}, \quad H = \begin{bmatrix}R_1\\&Q_2\\&&R_2\\&&&Q_3\\&&&&\ddots\\&&&&&R_{N-1}\\&&&&&&Q_N\end{bmatrix}\\
C&= \begin{bmatrix}B_1 & -I\\ &A_2&B_2&-I\\&&&A_3&B_3&-I\\&&&&&&\ddots\\&&&&&&&A_{N-1}&B_{N-1}&-I\end{bmatrix},\quad \mathbf{d}=\begin{bmatrix}-A\mathbf{x}_1\\ 0\\0\\\vdots\\0\end{bmatrix}
\end{aligned}$$

从而有

$$\begin{aligned}\frac{1}{2}\mathbf{z}^{\top}H\mathbf{z} &= \frac{1}{2}\sum_{i=1}^{N-1}\mathbf{u}_i^{\top}R_i\mathbf{u}_i + \frac{1}{2}\sum_{j=2}^{N}\mathbf{x}_j^{\top}Q_j\mathbf{x}_j\\
C\mathbf{z} &= \begin{bmatrix}B_1\mathbf{u}_1-\mathbf{x}_2\\A_2\mathbf{x}_2+B_2\mathbf{u}_2-\mathbf{x}_3\\A_3\mathbf{x}_3+B_3\mathbf{u}_3-\mathbf{x}_4\\
\vdots\\A_{N-1}\mathbf{x}_{N-1}+B_{N-1}\mathbf{u}_{N-1}-\mathbf{x}_N\end{bmatrix} = \mathbf{d}
\end{aligned}$$

于是，原问题就转化为：

$$\begin{aligned}\min_{\mathbf{z}}\quad &J = \frac{1}{2}\mathbf{z}^{\top}H\mathbf{z}\\
\mathrm{s.t.}\quad &C\mathbf{z}-\mathbf{d} = \mathbf{0}\end{aligned}$$

我们写出其拉格朗日方程

$$L(\mathbf{z},\lambda) = \frac{1}{2}\mathbf{z}^{\top}H\mathbf{z}+\lambda(C\mathbf{z}-\mathbf{d})$$

进而写出其 KKT 条件：

$$\begin{aligned}
&\nabla_{\mathbf{z}}L = H\mathbf{z}+C^{\top}\lambda = \mathbf{0}\\
&\nabla_{\lambda}L = C\mathbf{z}-\mathbf{d} = \mathbf{0}\\
\implies &\begin{bmatrix}H &C^{\top}\\ C & \mathbf{0}\end{bmatrix}\begin{bmatrix}\mathbf{z}\\\lambda\end{bmatrix} = \begin{bmatrix}\mathbf{0}\\\mathbf{d}\end{bmatrix} \end{aligned}$$

求解以上线性方程，即得最优控制序列。

## Riccati 递归法

观察 QP 法所得，待求解的线性方程：

$$
\left[ \begin{array}{c|c}
\begin{matrix} R_1\\ & Q_2\\ && R_2\\ &&& Q_3\\
&&&& R3 \\ &&&&& Q4\\ &&&&&& \ddots\\
&&&&&&& R_{N-1}\\ &&&&&&&& Q_{N} \end{matrix}
&
\begin{matrix} B_1^{\top}\\ -I & A_2^{\top}\\
& B_2^{\top}\\ & -I & A_3^{\top}\\
&& B_3^{\top} & \ddots \\ &&&& A_{N-1}^{\top}\\ 
&&&&B_{N-1}^{\top}\\ &&&&-I \end{matrix}
\\\hline
\begin{matrix} B_1 & -I \\ & A_2 & B_2 & -I \\
&&& A_3 & B_3 & -I\\
&&&&& \ddots\\ &&&&&&A_{N-1}&B_{N-1}&-I \end{matrix}
& \mathbf{0}
\end{array} \right]
\left[ \begin{array}{c}
\begin{matrix} \mathbf{u}_1\\ \mathbf{x}_2\\ \mathbf{u}_2\\ \mathbf{x}_3\\ \mathbf{u}_3\\ \mathbf{x}_4\\ \vdots\\ \mathbf{u}_{N-1}\\ \mathbf{x}_{N} \end{matrix}
\\\hline
\begin{matrix} \lambda_1\\ \lambda_2\\ \lambda_3\\ \vdots\\ \lambda_{N-1} \end{matrix}
\end{array} \right]
=
\left[ \begin{array}{c}
\begin{matrix}0\\0\\0\\0\\0\\0\\ \vdots\\0\\0\end{matrix}
\\\hline
\begin{matrix}-A\mathbf{x}_1\\0\\0\\\vdots\\0\end{matrix}
\end{array} \right]
$$

式中 $\lambda_i \in \mathbb{R}^{n}$ . 该方程非常稀疏，具有很强的结构性，暗示我们可以通过某种特别的方式求解。

我们主要考虑方程的上半边。首先，考虑上半边最后一个式子：

$$Q_N\mathbf{x}_N-\lambda_{N-1} = \mathbf{0} \implies \lambda_{N-1} = Q_N\mathbf{x}_N$$
接下来是倒数第二个式子：

$$\begin{aligned}
R_{N-1}\mathbf{u}_{N-1}+B_{N-1}^{\top}\lambda_{N-1} = 0\\
\overset{代入上式}\implies R_{N-1}\mathbf{u}_{N-1}+B^{\top}_{N-1}Q_N\mathbf{x}_N = 0\\
\overset{代入状态方程}\implies R_{N-1}\mathbf{u}_{N-1}+B^{\top}_{N-1}Q_N(A_{N-1}\mathbf{x}_{N-1}+B_{N-1}\mathbf{u}_{N-1}) = 0\\
\implies\mathbf{u}_{N-1} = -\left(R_{N-1}+B_{N-1}^{\top}Q_NB_{N-1}\right)^{-1}B_{N-1}^{\top}Q_NA_{N-1}\mathbf{x}_{N-1}\\
\overset{定义为}\implies\mathbf{u}_{N-1} = -K_{N-1}\mathbf{x}_{N-1}
\end{aligned}$$

然后是倒数第三个式子：

$$\begin{aligned}Q_{N-1}\mathbf{x}_{N-1}-\lambda_{N-2}+A_{N-1}^{\top}\lambda_{N-1} = 0\\
\implies Q_{N-1}\mathbf{x}_{N-1}-\lambda_{N-2}+A^{\top}_{N-1}Q_N\mathbf{x}_N = 0\\
\implies Q_{N-1}\mathbf{x}_{N-1}-\lambda_{N-2}+A^{\top}_{N-1}Q_N(A_{N-1}\mathbf{x}_{N-1}+B_{N-1}\mathbf{u}_{N-1}) = 0\\
\implies \lambda_{N-2} = \left[Q_{N-1}+A_{N-1}^{\top}Q_N(A_{N-1}-B_{N-1}K_{N-1})\right]\mathbf{x}_{N-1}\\
\overset{定义为}\implies\lambda_{N-2} = P_{N-1}\mathbf{x}_{N-1}
\end{aligned}$$

再来两个，倒数第四个式子：

$$\begin{aligned}
R_{N-2}\mathbf{u}_{N-2} + B_{N-2}^{\top}\lambda_{N-2} = 0\\
\implies R_{N-2}\mathbf{u}_{N-2} + B_{N-2}^{\top}P_{N-1}\mathbf{x}_{N-1} = 0\\
\implies R_{N-2}\mathbf{u}_{N-2} + B_{N-2}^{\top}P_{N-1}(A_{N-2}\mathbf{x}_{N-2}+B_{N-2}\mathbf{u}_{N-2}) = 0\\
\implies \mathbf{u}_{N-2} = -\left(R_{N-2}+B^{\top}_{N-2}P_{N-1}B_{N-2}\right)^{-1}B_{N-2}^{\top}P_{N-1}A_{N-2}\mathbf{x}_{N-2}\\
\implies \mathbf{u}_{N-2} = -K_{N-2}\mathbf{x}_{N-2}
\end{aligned}$$

倒数第五个式子：

$$\begin{aligned}
Q_{N-2}\mathbf{x}_{N-2}-\lambda_{N-3}+A_{N-2}^{\top}\lambda_{N-2} = 0\\
\implies Q_{N-2}\mathbf{x}_{N-2}-\lambda_{N-3}+A_{N-2}^{\top}P_{N-1}\mathbf{x}_{N-1} = 0\\
\implies Q_{N-2}\mathbf{x}_{N-2}-\lambda_{N-3}+A_{N-2}^{\top}P_{N-1}(A_{N-2}\mathbf{x}_{N-2}+B_{N-2}\mathbf{u}_{N-2}) = 0\\
\implies Q_{N-2}\mathbf{x}_{N-2}-\lambda_{N-3}+A_{N-2}^{\top}P_{N-1}(A_{N-2}-B_{N-2}K_{N-2})\mathbf{x}_{N-2} = 0\\
\implies \lambda_{N-3}=\left[Q_{N-2}+A_{N-2}^{\top}P_{N-1}(A_{N-2}-B_{N-2}K_{N-2})\right]\mathbf{x}_{N-2}\\
\implies \lambda_{N-3} = P_{N-2}\mathbf{x}_{N-2}
\end{aligned}$$

综合以上结果，能总结出反向递归的递推式：

$$\begin{aligned}
K_k &= \left(R_k+B_k^{\top}P_{k+1}B_k\right)^{-1}B_k^{\top}P_{k+1}A_k\\
P_k &= Q_k+ A_k^{\top}P_{k+1}(A_k-B_kK_k)\\
P_N &= Q_N\\
\mathbf{u}_k &= -K_k \mathbf{x}_k
\end{aligned}$$

已知 $A,B,Q,R$ 矩阵后，即可通过上述规律解得每个时刻的 **负反馈系数** ，从而根据当前状态实时计算负反馈控制指令。

Riccati 递归有以下优势：

1. QP 法解一个逆矩阵，时间复杂度是 $O(N^3(n^3+m^3))$ ，而 Riccati 递归法的时间复杂度是 $O(N(n^3+m^3))$ ；
2. Riccati 递归法最后得到一个负反馈控制律，适用于任意初始条件，并且能够处理一定的扰动、测量噪声或模型误差；
3. 对于 LTI (Linear time invariant) 系统，当 N 足够大时， $P_k$ 和 $K_k$ 都会随着 $k$ 的减小而收敛。此时 $P_{k}=P_{k+1}=P_{\infty}$ ，即无限时域解。因为时不变系统 $A,B$ 不变，Riccati 递归法的控制律与初始条件无关，所以在此情况下，确定了 $Q,R$ 后，**最终控制律是一个系数确定的负反馈控制** ！
4. 更妙的是，对于能控的 LTI 系统， Riccati 递归法给出的系数矩阵 $K_k$ 能保证控制效果是稳定的！
