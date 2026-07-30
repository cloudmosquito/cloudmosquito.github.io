# Luenberger 观测器

龙伯格观测器（Luenberger Observer）是一种典型的全维观测器。

## 连续形式

考虑连续线性系统

$$\begin{aligned}
\dot{\mathbf{x}} &= A\mathbf{x}+B\mathbf{u}\\
\mathbf{y} &= C\mathbf{x} + D\mathbf{u}
\end{aligned}$$

用 $\hat{\mathbf{x}}$ 和 $\hat{\mathbf{y}}$ 分别表示状态量和输出量的观测值，则有最简单的开环状态估计器：

$$\begin{aligned}
\dot{\hat{\mathbf{x}}} &= A\hat{\mathbf{x}}+B\mathbf{u}\\
\hat{\mathbf{y}} &= C\hat{\mathbf{x}} + D\mathbf{u}
\end{aligned}$$

![](./Luenberger%20观测器.assets/Luenberger观测器.jpg)

由于系统状态初值 $\mathbf{x}_0$ 往往难以获得，开环观测器必定存在误差。考虑将其扩展成闭环形式：

$$\begin{aligned}
\dot{\hat{\mathbf{x}}} &= A\hat{\mathbf{x}}+B\mathbf{u}+L(\mathbf{y}-\hat{\mathbf{y}})\\
\hat{\mathbf{y}} &= C\hat{\mathbf{x}} + D\mathbf{u}
\end{aligned}$$

以上就是针对连续线性系统的龙伯格观测器形式，其中 $L$ 是可调整的增益矩阵。

### 增益矩阵取值

我们接下来讨论，连续线性系统的龙伯格观测器的增益矩阵 $L$ 需满足何种条件，才能使得观测结果收敛至真值。

用 $\tilde{\mathbf{x}} = \mathbf{x}-\hat{\mathbf{x}}$ 表示状态量的观测误差，有

$$\begin{aligned}
\dot{\tilde{\mathbf{x}}} &= A\mathbf{x}+B\mathbf{u}-A\hat{\mathbf{x}}-B\mathbf{u}-L(\mathbf{y}-\hat{\mathbf{y}})\\
&= A\tilde{\mathbf{x}}-LC\tilde{\mathbf{x}}\\
&= (A-LC)\tilde{\mathbf{x}}
\end{aligned}$$

因此，当系统能观时，只需要选择合适的增益矩阵 $L$ 使得 $A-LC$ 的所有特征值均具有负实部，即可保证 $\tilde{\mathbf{x}}\rightarrow \mathbf{0}$ 。

## 离散形式

根据 [微分方程的离散化](../自动控制原理/微分方程的离散化.md) 对连续系统进行采样周期 $T_s$ 的精确离散化，得到

$$\begin{aligned}
\mathbf{x}[k+1]&=A_d\mathbf{x}[k]+B_d\mathbf{u}[k]\\
\mathbf{y}[k]&=C\mathbf{x}[k]+D\mathbf{u}[k]
\end{aligned}$$

其中，

$$A_d = e^{AT_s},B_d=\int_{0}^{T_s} e^{As}B\mathrm{d}s.$$

从而有龙伯格观测器的离散形式：

$$\begin{aligned}
\hat{\mathbf{x}}[k+1] &= A_d\hat{\mathbf{x}}[k]+B_d\mathbf{u}[k]+L_d(\mathbf{y}[k]-\hat{\mathbf{y}}[k])\\
&= A_d\hat{\mathbf{x}}[k]+B_d\mathbf{u}[k]+L_d(\mathbf{y}[k]-C\hat{\mathbf{x}}[k]-D\mathbf{u}[k])
\end{aligned}$$