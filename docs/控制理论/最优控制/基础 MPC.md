# 基础 MPC

考虑离散线性系统

$$\begin{aligned}\mathbf{x}_{k+1} &= A\mathbf{x}_k + B\mathbf{u}_k\\
\mathbf{z}_k &= C\mathbf{x}_k\end{aligned}$$

我们假设：

1. 系统结构完全已知，即矩阵 $A,B,C$ 已知；
2. 系统状态完全可知，即系统状态 $\mathbf{x}_k$ 已知。

需要注意，$\mathbf{z}_k$ 是系统实际输出，而非输出的测量结果。

接下来有两个重要的参数，预测窗口长度 $f$ 和控制窗口长度 $v$ ，满足 $v\le f$ .

MPC 的目标是，在 $k$ 时刻，计算后续一段时间的控制输入 $\mathbf{u}_{k+i|k},\ i=0,1,2,\cdots,v-1$ ，使得预测窗口内的系统输出 $\mathbf{z}_{k+i|k},\ i=1,2,3,\cdots, f$ 与期望轨迹的差异最小。

我们假设，在控制窗口外的 $k+v$ 时刻及后续时刻，控制输入保持不变，即

$$\mathbf{u}_{k+v-1|k} = \mathbf{u}_{k+v|k} = \mathbf{u}_{k+v+1|k} =\cdots = \mathbf{u}_{k+f|k}$$

MPC 不会采纳控制窗口里的所有计算结果，只会采纳第一个 $u_{k|k}$ 作为 $k$ 时刻的控制输入。在 $k+1$ 时刻，MPC 重新计算。

## 系统输出的表达式

本小节，我们计算 $\mathbf{z}_{k+i|k}, \ i=1,2,3,\cdots,f$ 的表达式。

由于矩阵 $C$ 是定的，我们先计算 $\mathbf{x}_{k+i|k}$ ：

$$\begin{aligned}
\mathbf{x}_{k+1|k} &= A\mathbf{x}_k+B\mathbf{u}_{k|k}\\
\mathbf{x}_{k+2|k} &= A\mathbf{x}_{k+1|k} + B\mathbf{u}_{k+1|k}\\
&= A^2\mathbf{x}_k + AB\mathbf{u}_{k|k} + B\mathbf{u}_{k+1|k}\\
\mathbf{x}_{k+3|k} &= A\mathbf{x}_{k+2|k} + B\mathbf{u}_{k+2|k}\\
&= A^3\mathbf{x}_k + A^{2}B\mathbf{u}_{k|k}+AB\mathbf{u}_{k+1|k}+B\mathbf{u}_{k+2|k}\\
\cdots\\
\mathbf{x}_{k+v|k} &= A^{v}\mathbf{x}_k+\begin{bmatrix}A^{v-1}B & A^{v-2}B & \cdots & AB & B\end{bmatrix}\begin{bmatrix}\mathbf{u}_{k|k}\\ \mathbf{u}_{k+1|k}\\ \vdots \\ \mathbf{u}_{k+v-1|k}\end{bmatrix}\\
\mathbf{x}_{k+v+1|k} &= A\mathbf{x}_{k+v|k} + B\mathbf{u}_{k+v|k}\\
&= A\mathbf{x}_{k+v|k} + B\mathbf{u}_{k+v-1|k}\\
&= A^{v+1}\mathbf{x}_k + \begin{bmatrix}A^{v}B & A^{v-1}B & \cdots & A^2B & AB+B\end{bmatrix}\begin{bmatrix}\mathbf{u}_{k|k}\\ \mathbf{u}_{k+1|k}\\ \vdots \\ \mathbf{u}_{k+v-1|k}\end{bmatrix}\\
\mathbf{x}_{k+v+2|k} &= A\mathbf{x}_{k+v+1|k}+B\mathbf{u}_{k+v+1|k}\\
&= A^{v+2}\mathbf{x}_k + \begin{bmatrix}A^{v+1}B & A^{v}B & \cdots & A^3B & (A^2+A+I)B\end{bmatrix}\begin{bmatrix}\mathbf{u}_{k|k}\\\mathbf{u}_{k+1|k}\\\vdots\\ \mathbf{u}_{k+v-1|k}\end{bmatrix}\\
\cdots\\
\mathbf{x}_{k+f|k} &= A^{f}\mathbf{x}_k+\begin{bmatrix}A^{f-1}B & A^{f-2}B& \cdots & A^{f-v+1}B & (A^{f-v}+A^{f-v-1}+\cdots+A+I)B\end{bmatrix}\begin{bmatrix}\mathbf{u}_{k|k}\\\mathbf{u}_{k+1|k}\\\vdots\\ \mathbf{u}_{k+v-1|k}\end{bmatrix}
\end{aligned}$$

我们引入符号 $\bar{A}_{f,v} = A^{f-v} + A^{f-v-1} + \cdots + A + I$ 简化以上表达式。将以上所有表达式合并，我们得到：

$$\begin{bmatrix}\mathbf{x}_{k+1|k}\\ \mathbf{x}_{k+2|k}\\ \vdots\\ \mathbf{x}_{k+v|k}\\ \mathbf{x}_{k+v-1|k}\\ \vdots\\ \mathbf{x}_{k+f|k}\end{bmatrix} = \begin{bmatrix}A\\A^2\\\vdots\\A^v\\A^{v+1}\\\vdots\\A^{f}\end{bmatrix}\mathbf{x}_k + \begin{bmatrix}B\\ AB & B\\ \vdots & \vdots\\ A^{v-1}B & A^{v-2}B & \cdots & AB & B\\A^{v}B & A^{v-1}B & \cdots & A^{2}B & \bar{A}_{v+1,v}B\\\vdots & \vdots & \cdots & \vdots& \vdots\\A^{f-1}B & A^{f-2}B & \cdots & A^{f-v+1}B & \bar{A}_{f,v}B\end{bmatrix}\begin{bmatrix}\mathbf{u}_{k|k}\\ \mathbf{u}_{k+1|k}\\\vdots\\\mathbf{u}_{k+v-1|k}\end{bmatrix}$$

进而，考虑 $\mathbf{z}_k = C\mathbf{x}_k$ ，我们得到

$$\begin{bmatrix}\mathbf{z}_{k+1|k}\\ \mathbf{z}_{k+2|k}\\ \vdots\\ \mathbf{z}_{k+v|k}\\ \mathbf{z}_{k+v-1|k}\\ \vdots\\ \mathbf{z}_{k+f|k}\end{bmatrix} = \begin{bmatrix}CA\\ CA^2\\\vdots\\ CA^v\\ CA^{v+1}\\\vdots\\ CA^{f}\end{bmatrix}\mathbf{x}_k + \begin{bmatrix}CB\\ CAB & CB\\ \vdots & \vdots\\ CA^{v-1}B & CA^{v-2}B & \cdots & CAB & CB\\ CA^{v}B & CA^{v-1}B & \cdots & CA^{2}B & C\bar{A}_{v+1,v}B\\\vdots & \vdots & \cdots & \vdots& \vdots\\ CA^{f-1}B & CA^{f-2}B & \cdots & CA^{f-v+1}B & C\bar{A}_{f,v}B\end{bmatrix}\begin{bmatrix}\mathbf{u}_{k|k}\\ \mathbf{u}_{k+1|k}\\\vdots\\\mathbf{u}_{k+v-1|k}\end{bmatrix}$$

简洁起见，我们将上式定义为

$$\mathbf{z} = O\mathbf{x}_k + M\mathbf{u}$$

## 优化目标

我们期望的输出参考轨迹用向量的形式表示为

$$\mathbf{z}^d = \begin{bmatrix}\mathbf{z}^d_{k+1} & \mathbf{z}^{d}_{k+2} & \cdots & \mathbf{z}_{k+f}^{d}\end{bmatrix}^{\top}$$

最直接的优化目标就是

$$\min\left\|\mathbf{z}-\mathbf{z}^{d}\right\|_2 = \min \left(\mathbf{z}-\mathbf{z}^d\right)^{\top}\left(\mathbf{z}-\mathbf{z}^{d}\right)$$

这一优化目标没有对控制输入 $\mathbf{u}$ 加以惩罚限制，可能导致 $\mathbf{u}$ 使执行器饱和，甚至无法实施。

此外，不同时刻的轨迹误差应当具有不同的权重。直观来看，我们希望靠近现在的轨迹误差更小，以后的轨迹误差适当放宽。因为当前 MPC 输出的最终控制指令仅为 $\mathbf{u}_{k|k}$ ，以后的轨迹误差可以交由以后的 MPC 计算、优化。

我们将控制输入的惩罚项表示为

$$\begin{aligned}
J_{\mathbf{u}} &= \mathbf{u}_{k|k}^{\top}Q_0\mathbf{u}_{k|k} + (\mathbf{u}_{k+1|k}-\mathbf{u}_{k|k})^\top Q_1 (\mathbf{u}_{k+1|k}-\mathbf{u}_{k|k})\\
&\ \ \ \ \ + \cdots + (\mathbf{u}_{k+v-1|k}-\mathbf{u}_{k+v-2|k})^\top Q_{v-1} (\mathbf{u}_{k+v-1|k}-\mathbf{u}_{k+v-2|k})\\
&= \left(\begin{bmatrix}I\\ -I & I\\ \vdots & \vdots &\cdots\\0 & 0 & \cdots & -I & I\end{bmatrix}\mathbf{u}\right)^\top \begin{bmatrix}Q_0\\ &Q_1\\ & & \cdots\\ &&&&Q_{v-1}\end{bmatrix} \left(\begin{bmatrix}I\\ -I & I\\ \vdots & \vdots &\cdots\\0 & 0 & \cdots & -I & I\end{bmatrix}\mathbf{u}\right)\\
&= (W_1\mathbf{u})^\top W_2 W_1\mathbf{u}\\
&= \mathbf{u}^\top W_1^{\top}W_2W_1 \mathbf{u}\\
&= \mathbf{u}^\top W_3 \mathbf{u}
\end{aligned}$$

> 这里的 $W_2,W_3$ 是一个对称矩阵。

该惩罚项的设计思路是限制控制输入的变化率。

$$\begin{aligned}
J_{\mathbf{z}} &= \left(\mathbf{z}-\mathbf{z}^d\right)^{\top} \begin{bmatrix}P_1\\ &P_2\\&&\cdots\\&&&P_f\end{bmatrix} \left(\mathbf{z}-\mathbf{z}^d\right)\\
&= \left(O\mathbf{x}_k+M\mathbf{u}-\mathbf{z}^{d}\right)^{\top}W_4 \left(O\mathbf{x}_k+M\mathbf{u}-\mathbf{z}^{d}\right)\\
&= (M\mathbf{u}-\mathbf{s})^\top W_4 (M\mathbf{u}-\mathbf{s})
\end{aligned}$$

> 这里的 $W_4$ 是一个对称矩阵。

其中， $\mathbf{s} = \mathbf{z}^d - O\mathbf{x}_k$ .

最终，我们的优化目标是 $\min J_{\mathbf{u}}+J_{\mathbf{z}}$ .

$$\begin{aligned}
\min\ & J_{\mathbf{u}}+J\mathbf{z}\\
=\min\ & \mathbf{u}^\top W_3\mathbf{u}+(M\mathbf{u}-\mathbf{s})^\top W_4(M\mathbf{u}-s)\\
=\min\ &\mathbf{u}^\top W_3\mathbf{u}+ \mathbf{u}^{\top}M^{\top}W_4M\mathbf{u}- \mathbf{u}^{\top}M^{\top}W_4 \mathbf{s}-\mathbf{s}^{\top}W_4M\mathbf{u}+\mathbf{s}^\top W_4\mathbf{s}
\end{aligned}$$

接下来，将 $J_{\mathbf{u}}+J_{\mathbf{z}}$ 关于 $\mathbf{u}$ 求偏导，求得最佳 $\mathbf{u}$ 

$$\begin{aligned}
\frac{\partial J_{\mathbf{u}}+J_{\mathbf{z}}}{\mathbf{u}} &= \mathbf{u}^{\top}\left(W_3+W_3^{\top}\right) + \mathbf{u}^{\top}\left(M^{\top}W_4M+M^{\top}W_4^{\top}M\right)- \mathbf{s}^{\top}W_4^{\top}M\\
&\ \ \ \ \ \ -\mathbf{s}^{\top}W_4M\\
&= 2\mathbf{u}^{\top}\left(W_3+M^{\top}W_4M\right)-2\mathbf{s}^{\top}W_4M
\end{aligned}$$

求 $\dfrac{\partial J_{\mathbf{u}}+J_{\mathbf{z}}}{\partial\mathbf{u}} = 0$ 时的最佳输入 $\mathbf{u}$ ： 

$$\begin{aligned}
\frac{\partial J_{\mathbf{u}}+J_{\mathbf{z}}}{\partial\mathbf{u}} &= 0\\
2\mathbf{u}^{\top}\left(W_3+M^{\top}W_4M\right)-2\mathbf{s}^{\top}W_4M &= 0\\
\mathbf{u}^{\top}\left(W_3+M^{\top}W_4M\right)&=\mathbf{s}^{\top}W_4M\\
\left(W_3+M^{\top}W_4M\right)\mathbf{u} &= M^{\top}W_4\mathbf{s}\\
\mathbf{u} &= \left(W_3+M^{\top}W_4M\right)^{-1}M^{\top}W_4\mathbf{s}
\end{aligned}$$