# KF & EKF

## 1 任务定义

考虑线性离散系统状态空间方程

$$\begin{aligned}
\mathbf{x}_k &= A\mathbf{x}_{k-1}+B\mathbf{u}_{k-1}+\mathbf{w}_{k-1}\\
\mathbf{z}_k &= H\mathbf{x}_k + \mathbf{v}_k
\end{aligned}$$

这里的 $\mathbf{w}_{k-1}$ 和 $\mathbf{v}_k$ 都是噪声，服从如下正态分布

$$
\mathbf{w}_k \sim \mathcal{N}(\mathbf{0}, Q_k) \ \ \ \ \ \ \ \ \ \ \mathbf{v}_k \sim \mathcal{N}(\mathbf{0},R_k)
$$

其中，我们已知的是系统结构 $A,B,H$ ，测量结果 $\mathbf{z}_k$ ，控制输入 $\mathbf{u}_k$ 以及协方差矩阵 $Q_k$ 和  $R_k$ 。目的是求得系统状态 $\mathbf{x}_k$ 的最优估计。（这里的 $A,H$ 都是方阵， $B$ 可以不是方阵。）

## 2 先验估计与测量

首先，在获得上一时刻的估计值 $\hat{\mathbf{x}}_{k-1}$ 后，可以计算得到本时刻系统状态的先验估计 $\hat{\mathbf{x}}_k^{-}$  （右上角的负号代表先验）：

$$
\hat{\mathbf{x}}_k^{-} = A\hat{\mathbf{x}}_{k-1} + B\mathbf{u}_{k-1} \tag{1}
$$

其次，在获得当前时刻的测量值后，可以计算得到本时刻系统状态的测量结果 $\hat{\mathbf{x}}_{k,meas}$

$$
\hat{\mathbf{x}}_{k,meas} = H^{-1}\mathbf{z}_k \tag{2}
$$

> 【注意】实际系统中，$H$ 很可能是不可逆的，即无法通过测量值得知状态量。此处假设 $H$ 是可逆的，仅为了方便理解下文数据融合的思路。

## 2 数据融合与 Kalman Gain

### 2.1 宕开一笔——数据融合

考虑这样两个满足正态分布的测量结果，一个均值 $z_1 = 6.5$ ，标准差 $\sigma_1 = 0.2$ ；另一个均值 $z_2 = 7.3$ ，标准差 $\sigma_2 = 0.4$ 。请问如何根据这两个测量结果，得到一个最优的测量估计值？

首先一个最基本的思路是将这两个数据加权求和，即

$$
\hat{z} = z_1+k(z_2-z_1)
$$

这里的 $k \in [0,1]$ ，表示对 $z_2$ 的信任程度 / 对 $z_1$ 的不信任程度。接下来的问题是， $k$ 取何值时，估计是最优的？

我们如何定义"最优性"？让我们回顾一下测量结果中"标准差"的含义。以测量结果 1 为例，由于满足正态分布，标准差代表真值有 68.27% 的概率落在 \[6.3, 6.7\] 之间，有 95.45% 的概率落在 \[6.1, 6.9\] 之间，有 99.73% 的概率落在 \[5.9, 7.1\] 之间。

我们的 $\hat{z}$ 可以理解为测量估计的均值，也就是这些区间的中间值；若测量估计的标准差/方差越小，则测量估计与真值越接近，则估计越好。

也就是说，我们需要找到一个 $k$ 使得 $\hat{z}$ 的方差最小。

$$\begin{aligned}
\mathrm{Var}(\hat{z}) &= \mathrm{Var}[(1-k)z_1] + \mathrm{Var}[kz_2]\\
&= (1-k)^2\mathrm{Var}[z_1]+k^2\mathrm{Var}[z_2]\\
&= (1-k)^2\sigma_1^2 + k^2\sigma_2^2\\
&= \left(\sigma_1^2+\sigma_2^2\right)k^2 -2\sigma_1^2k + \sigma_1^2
\end{aligned}$$

这是一个关于 $k$ 的二次函数，其极小值点在 $k = \dfrac{\sigma_1^2}{\sigma_1^2 + \sigma_2^2}$ . 因此，最优测量估计为

$$
\hat{z} = \frac{\sigma_2^2}{\sigma_1^2+\sigma_2^2}z_1 + \frac{\sigma_1^2}{\sigma_1^2+\sigma_2^2}z_2
$$

### 2.2 回归正题——引出 Kalman Gain

这里的 (1) (2) 两式都没有考虑噪声。我们采用如下的常见方式融合这两个数据，得到系统状态的后验估计：

$$
\hat{\mathbf{x}}_k = \hat{\mathbf{x}}_k^{-} + (K_kH)\left(\hat{\mathbf{x}}_{k,meas}-\hat{\mathbf{x}}_k^{-}\right)
$$

这里的 $K_kH$ 是个对角方阵，对角线上每个元素取值都在 $[0,1]$ ，表示对测量值的信任程度。从而有

$$
\hat{\mathbf{x}}_k = \hat{\mathbf{x}}_k^{-}+K_k\left(\mathbf{z}_k-H\hat{\mathbf{x}}_k^{-}\right) \tag{3}
$$

> 之所以采用 $K_kH$ 作为系数矩阵，是为了消去 $\hat{\mathbf{x}}_{k,meas}$ 中多数情况下不成立的 $H^{-1}$ 一项。

从而有后验估计的误差值：

$$
\begin{aligned}\mathbf{e}_k &= \mathbf{x}_k - \hat{\mathbf{x}}_k\\
&=\mathbf{x}_k - \hat{\mathbf{x}}_k^{-}-K_k\left(\mathbf{z}_k-H\hat{\mathbf{x}}_k^{-}\right)\\
&= \mathbf{x}_k - \hat{\mathbf{x}}_k^{-}-K_k\left(H\mathbf{x}_k + \mathbf{v}_k-H\hat{\mathbf{x}}_k^{-}\right)\\
&= (I-K_kH)\left(\mathbf{x}_k-\hat{\mathbf{x}}_k^{-}\right)-K_k\mathbf{v}_k\\
&= (I-K_kH)\mathbf{e}_k^{-}-K_k\mathbf{v}_k
\end{aligned}$$

我们想要求得 $\mathbf{x}_k$ 的最优估计，也就是想要寻找一个最优的卡尔曼系数 $K_k$ ，使得误差 $\mathbf{e}_k$ 最小。

注意到误差满足正态分布，期望为 0 。设 $\mathbf{e}_k \sim \mathcal{N}(0,P_k)$ ，其中 $P_k$ 是协方差矩阵。

我们想使误差最小，其实是想使误差分布尽可能集中在 0 附近，即方差尽可能小，即 $\mathrm{tr}(P)$ 尽可能小。

> 怎么计算协方差矩阵？$\mathrm{VAR}(\mathbf{x}) = \mathbb{E}\left(\mathbf{x}\mathbf{x}^{\top}\right) -\mathbb{E}^2(\mathbf{x})$ .

现在，我们计算后验估计误差分布的协方差矩阵。注意到 $\mathbb{E}(\mathbf{e}_k) = 0$ ，从而有：

$$\begin{aligned}
P_k &= \mathbb{E}\left(\mathbf{e}_k\mathbf{e}_k^\top\right)\\
&= \mathbb{E}\left\{\left[(I-K_kH)\mathbf{e}_k^{-}-K_k\mathbf{v}_k\right]\left[(I-K_kH)\mathbf{e}_k^{-}-K_k\mathbf{v}_k\right]^\top\right\}\\
&= \mathbb{E}\left\{\left[(I-K_kH)\mathbf{e}_k^{-}-K_k\mathbf{v}_k\right]\left[\mathbf{e}_k^{-\top}(I-K_kH)^{\top}-\mathbf{v}_k^{\top}K_k^{\top}\right]\right\}\\
&= \mathbb{E}[(I-K_kH)\mathbf{e}_k^{-}\mathbf{e}_k^{-\top}(I-K_kH)^{\top}-(I-K_kH)\mathbf{e}_k^{-}\mathbf{v}_k^\top K_k^\top-\\
&\ \ \ \ \ \ K_k\mathbf{v}_k\mathbf{e}_k^{-\top}(I-K_kH)^{\top} + K_k\mathbf{v}_k\mathbf{v}_k^{\top}K_k^{\top}]\\
&= \mathbb{E}\left[(I-K_kH)\mathbf{e}_k^{-}\mathbf{e}_k^{-\top}(I-K_kH)^{\top}\right] - \mathbb{E}\left[(I-K_kH)\mathbf{e}_k^{-}\mathbf{v}_k^\top K_k^\top\right] - \\
&\ \ \ \ \ \ \mathbb{E}\left[K_k\mathbf{v}_k\mathbf{e}_k^{-\top}(I-K_kH)^{\top}\right] + \mathbb{E}\left[K_k\mathbf{v}_k\mathbf{v}_k^\top K_k^\top\right]
\end{aligned}$$

其中，注意到先验误差 $\mathbf{e}_k^{-}$ 和测量误差 $\mathbf{v}_k$ 是相互独立的，因此有

$$\begin{aligned}
\mathbb{E}\left[(I-K_kH)\mathbf{e}_k^{-}\mathbf{v}_k^\top K_k^\top\right] &= (I-K_kH)\mathbb{E}\left(\mathbf{e}_k^{-}\mathbf{v}_k^\top\right) K_k^\top\\
&= (I-K_kH)\mathbb{E}\left(\mathbf{e}_k^{-}\right)\mathbb{E}\left(\mathbf{v}_k^\top\right) K_k^\top\\
&= \mathbf{0}
\end{aligned}$$

此外，因为 $\mathbf{e}_k^{-}$ 和 $\mathbf{v}_k$ 的期望都为 $\mathbf{0}$ ，则有

$$
\mathbb{E}\left(\mathbf{e}_k^{-}\mathbf{e}_k^{-\top}\right) = P_k^{-} \ \ \ \ \ \ \ \ \ \ \mathbb{E}\left(\mathbf{v}_k\mathbf{v}_k^{\top}\right) = R_k
$$

进而可以推得：

$$
\begin{aligned}P_k &= (I-K_kH)\mathbb{E}\left(\mathbf{e}_k^{-}\mathbf{e}_k^{-\top}\right)(I-K_kH)^{\top} + K_k\mathbb{E}\left(\mathbf{v}_k\mathbf{v}_k^\top\right)K_k^\top\\
&= (I-K_kH)P_k^{-}(I-K_kH)^{\top} + K_kR_kK_k^\top\end{aligned}\tag{Joseph}
$$

上式被称为 Joseph 形式。我们继续化简：

$$\begin{aligned}
P_k &= (I-K_kH)P_k^{-}(I-K_kH)^{\top} + K_kR_kK_k^\top\\
&= (I-K_kH)P_k^{-}\left(I-H^\top K_k^\top\right) + K_kR_kK_k^\top\\
&= (P_k^{-}-K_kHP_k^{-})\left(I-H^\top K_k^\top\right) + K_kR_kK_k^\top\\
&= P_k^{-}-P_k^{-}H^\top K_k^{\top}-K_kHP_k^{-}+K_kHP_k^{-}H^\top K_k^{\top} + K_kR_kK_k^\top
\end{aligned}$$

注意到 $P_k^{-}H^\top K_k^\top = (K_kHP_k^{-})^{\top}$ ，所以两者的逆相等。从而可以计算协方差矩阵的迹：

$$\begin{aligned}
\mathrm{tr}(P_k) &= \mathrm{tr}\left(P_k^{-}\right)-2\mathrm{tr}\left(K_kHP_k^{-}\right) + \mathrm{tr}\left(K_kHP_k^{-}H^\top K_k^\top\right) + \mathrm{tr}\left(K_kR_kK_k^\top\right)
\end{aligned}$$

接下来，我们要找到卡尔曼系数 $K_k$ 的最优值，让误差分布的协方差矩阵的迹最小。

---

【引理】 $\frac{\partial \mathrm{tr}\left(AB\right)}{\partial A} = \frac{\partial \mathrm{tr}\left(B^\top A^\top\right)}{\partial A}=B^\top$ ，其中 $A\in \mathbb{R}^{m\times n}, B \in \mathbb{R}^{n\times m}$ 。

【证明】设

$$\begin{aligned}
A_{m\times n} = \begin{bmatrix}\mathbf{a}_{1,1\times n}\\ \mathbf{a}_2\\ \vdots\\ \mathbf{a}_m\end{bmatrix} = \begin{bmatrix}a_{11} & a_{12} & \cdots & a_{1n}\\ \vdots & \vdots & & \vdots\\ a_{m1} & a_{m2} & \cdots & a_{mn}\end{bmatrix}\\
B_{n\times m} = \begin{bmatrix}\mathbf{b}_{1,n\times1} & \mathbf{b}_2 & \cdots & \mathbf{b}_m\end{bmatrix} =  \begin{bmatrix}b_{11} & b_{12} & \cdots & b_{1m}\\ \vdots & \vdots & & \vdots\\ b_{n1} & b_{n2} & \cdots & b_{nm}\end{bmatrix}
\end{aligned}$$

则有 $\mathrm{tr}(AB) = \mathbf{a}_1\mathbf{b}_1 + \cdots + \mathbf{a}_m\mathbf{b}_m$ 。而 $\frac{\partial \mathrm{tr}(AB)}{\partial a_{ij}} = \frac{\partial \mathbf{a}_i\mathbf{b}_i}{\partial a_{ij}} = b_{ji}$ ，所以

$$\begin{aligned}
\frac{\partial \mathrm{tr}\left(AB\right)}{\partial A} &= \begin{bmatrix}\frac{\partial\mathrm{tr}\left(AB\right)}{\partial a_{11}} & \cdots & \frac{\partial\mathrm{tr}\left(AB\right)}{\partial a_{1m}}\\ \vdots & &\vdots\\ \frac{\partial\mathrm{tr}\left(AB\right)}{\partial a_{n1}} & \cdots & \frac{\partial\mathrm{tr}\left(AB\right)}{\partial a_{nm}}\end{bmatrix} = \begin{bmatrix}b_{11} & \cdots & b_{n1}\\ \vdots & & \vdots\\ b_{1m} & \cdots & b_{nm}\end{bmatrix}\\
&= B^\top\end{aligned}$$

【引理】 $\frac{\partial \mathrm{tr}\left(ABA^\top\right)}{\partial A}=A\left(B^\top+B\right)$  ，其中 $A,B \in \mathbb{R}^{n\times n}$ 。

【证明】

$$\begin{aligned}
\frac{\partial \mathrm{tr}\left(ABA^\top\right)}{\partial A} &= \frac{\mathrm{tr}\left[\mathrm{d}A\left(BA^\top\right)\right]}{\partial A} + \frac{\mathrm{tr}\left[(AB)\mathrm{d}A^\top\right]}{\partial A}\\
&= \left(BA^\top\right)^\top + (AB)\\
&= A\left(B^\top+B\right)
\end{aligned}$$

---

我们通过对误差分布的协方差矩阵的迹求导，求其极值点：

$$
\begin{aligned}
\frac{\partial \mathrm{tr}(P_k)}{\partial K_k} &= -2(HP_k^{-})^\top + \frac{\partial\mathrm{tr}\left(K_kHP_k^{-}H^\top K_k^\top\right)}{\partial K_k} + \frac{\partial\mathrm{tr}\left(K_kR_kK_k^\top\right)}{\partial K_k}\\
&= -2(HP_k^{-})^\top + K_k\left(HP_k^{-}H^\top + HP_k^{-\top}H^\top\right) + K_k\left(R_k+R_k^{\top}\right)\\
\end{aligned}
$$

注意到协方差矩阵 $P_k^{-}$ 和 $R_k$ 都是对称矩阵，因而有

$$
\frac{\partial \mathrm{tr}(P_k)}{\partial K_k} = -2P_k^{-} H^\top + 2K_kHP_k^{-}H^\top + 2K_kR_k = 0
$$

从而可以推导得到极小值点为

$$
K_k = P_k^{-} H^\top \left(HP_k^{-}H^\top + R_k\right)^{-1}
$$

这就是最优的卡尔曼增益（Kalman Gain）。

## 3 Kalman Gain 中的协方差

现在，唯一的问题是如何计算先验误差的协方差矩阵 $P_k^{-}$ 。根据定义，$P_k^{-} = \mathbb{E}\left[\mathbf{e}_k^{-}\mathbf{e}_k^{-\top}\right]$ 。

$$\begin{aligned}
\mathbf{e}_k^{-} &= \mathbf{x}_k - \hat{\mathbf{x}}_k^{-}\\
&= A\mathbf{x}_{k-1}+B\mathbf{u}_{k-1}+\mathbf{w}_{k-1} - (A\hat{\mathbf{x}}_{k-1} + B\mathbf{u}_{k-1})\\
&= A(\mathbf{x}_{k-1} - \hat{\mathbf{x}}_{k-1}) + \mathbf{w}_{k-1}\\
&= A\mathbf{e}_{k-1} + \mathbf{w}_{k-1}
\end{aligned}$$

将上式代入协方差矩阵定义，有

$$\begin{aligned}
P_k^{-} &= \mathbb{E}\left[(A\mathbf{e}_{k-1} + \mathbf{w}_{k-1})\left(\mathbf{e}_{k-1}^{\top}A^{\top}+\mathbf{w}_{k-1}^{\top}\right)\right]\\
&= \mathbb{E}\left[A\mathbf{e}_{k-1}\mathbf{e}_{k-1}^{\top}A^{\top}\right] + \mathbb{E}\left[A\mathbf{e}_{k-1}\mathbf{w}_{k-1}^{\top}\right] + \mathbb{E}\left[\mathbf{w}_{k-1}\mathbf{e}_{k-1}^\top A^\top\right] + \mathbb{E}\left[\mathbf{w}_{k-1}\mathbf{w}_{k-1}^\top\right]
\end{aligned}$$

注意到 $\mathbf{e}_{k-1}$ 描述第 k-1 次的状态估计误差，而 $\mathbf{w}_{k-1}$ 是第 k 次的过程噪声，两者相互独立，因而有：

$$\begin{aligned}
\mathbb{E}\left[A\mathbf{e}_{k-1}\mathbf{w}_{k-1}^\top\right] &= A\mathbb{E}[\mathbf{e}_{k-1}]\mathbb{E}\left[\mathbf{w}_{k-1}^\top\right]\\
&= A\mathbf{0}_{n\times 1}\mathbf{0}_{1\times n}\\
&= \mathbf{0}_{n\times n}
\end{aligned}$$

同理， $\mathbb{E}\left[\mathbf{w}_{k-1}\mathbf{e}_{k-1}^\top A^\top\right] = \mathbf{0}_{n\times n}$ 。因此，协方差矩阵可表示为：

$$\begin{aligned}
P_k^{-} &= A\mathbb{E}\left[\mathbf{e}_{k-1}\mathbf{e}_{k-1}^\top\right]A^\top + \mathbb{E}\left[\mathbf{w}_{k-1}\mathbf{w}_{k-1}^\top\right]\\
&= AP_{k-1}A^\top + Q_{k-1}
\end{aligned}$$

## 4 Kalman Filter 公式

### 4.1 预测过程

先验估计： $\hat{\mathbf{x}}_k^{-} = A\hat{\mathbf{x}}_{k-1} + B\mathbf{u}_{k-1}$

先验误差协方差： $P_k^{-} = AP_{k-1}A^\top + Q_{k-1}$

### 4.2 校正过程

卡尔曼增益： $K_k = P_k^{-} H^\top \left(HP_k^{-}H^\top + R_k\right)^{-1}$

后验估计： $\hat{\mathbf{x}}_k = \hat{\mathbf{x}}_k^{-}+K_k\left(\mathbf{z}_k-H\hat{\mathbf{x}}_k^{-}\right)$ （最终的估计结果）

更新后验误差的协方差矩阵：

$$\begin{aligned}
P_k &= P_k^{-}-P_k^{-}H^\top K_k^{\top}-K_kHP_k^{-}+K_kHP_k^{-}H^\top K_k^{\top} + K_kR_kK_k^\top\\
&= P_k^{-}-P_k^{-}H^\top K_k^{\top}-K_kHP_k^{-}+ K_k\left(HP_k^{-}H^\top + R_k\right)K_k^\top\\
&= P_k^{-}-P_k^{-}H^\top K_k^{\top}-K_kHP_k^{-}\\ &\ \ \ \ + P_k^{-} H^\top \left(HP_k^{-}H^\top + R_k\right)^{-1}\left(HP_k^{-}H^\top + R_k\right)K_k^\top\\
&= P_k^{-}-K_kHP_k^{-}\\
&= \left(I-K_kH\right)P_k^{-}
\end{aligned}$$

以及 Joseph 形式：

$$
P_k = (I-K_kH)P_k^{-}(I-K_kH)^{\top} + K_kR_kK_k^\top
$$

在工程实际中，Joseph 形式是更受推荐的形式，因其显式保证了计算结果的对称正定性，避免了浮点舍入误差或强非线性对计算结果正定对称性的干扰。

## 5 初值问题

TODO

## 6 EKF 扩展卡尔曼滤波器

之前我们所考虑的都是线性系统，那如果是更常见的非线性系统呢？

$$\begin{aligned}
\mathbf{x}_k &= f(\mathbf{x}_{k-1},\mathbf{u}_{k-1},\mathbf{w}_{k-1})\\
\mathbf{z}_k &= h(\mathbf{x}_k, \mathbf{v}_k)
\end{aligned}$$

大问题：**正态分布的随机变量通过非线性映射后，就不再是正态分布的了**。

常见的解决方法：在工作点（Operating Point）处，对非线性系统进行线性化。但我们实际上无法获知真实工作点，所以常见的做法是在上一时刻的后验估计 $\hat{\mathbf{x}}_{k-1}, \hat{\mathbf{w}}_{k-1}$ 处对系统进行线性化。

首先考虑模型动力学部分：

$$\begin{aligned}
\mathbf{x}_k = f(\hat{\mathbf{x}}_{k-1},\mathbf{u}_{k-1},\hat{\mathbf{w}}_{k-1}) &+ \frac{\partial f}{\partial \mathbf{x}}|_{\hat{\mathbf{x}}_{k-1}, \mathbf{u}_{k-1}, \hat{\mathbf{w}}_{k-1}}(\mathbf{x}_{k-1} - \hat{\mathbf{x}}_{k-1})\\ &+ \frac{\partial f}{\partial \mathbf{w}}|_{\hat{\mathbf{x}}_{k-1}, \mathbf{u}_{k-1}, \hat{\mathbf{w}}_{k-1}}(\mathbf{w}_{k-1} - \hat{\mathbf{w}}_{k-1})
\end{aligned}$$

> Q：为什么不对控制输入 $\mathbf{u}_{k-1}$ 求偏导？
> A：因为求完之后 $\mathbf{u}_{k-1} - \mathbf{u}_{k-1} = 0$ ，即我们所知的控制输入必是真实工作点的控制输入。

由于我们对噪声信息基本没什么了解，所以直接将估计值设为 0 ，即 $\hat{\mathbf{w}}_{k-1} = \mathbf{0}$ . 进而有：

$$
\mathbf{x}_k = f(\hat{\mathbf{x}}_{k-1}, \mathbf{u}_{k-1}, \mathbf{0}) + \frac{\partial f}{\partial \mathbf{x}}|_{\hat{\mathbf{x}}_{k-1}, \mathbf{u}_{k-1}, \mathbf{0}}(\mathbf{x}_{k-1} - \hat{\mathbf{x}}_{k-1}) + \frac{\partial f}{\partial \mathbf{w}}|_{\hat{\mathbf{x}}_{k-1}, \mathbf{u}_{k-1}, \mathbf{0}}\mathbf{w}_{k-1}
$$

令 $\tilde{\mathbf{x}}_k = f(\hat{\mathbf{x}}_{k-1}, \mathbf{u}_{k-1}, \mathbf{0})$ ，在此处对测量模型进行线性化，同理有

$$
\mathbf{z}_k = h(\tilde{\mathbf{x}}_k, \mathbf{0}) + \frac{\partial h}{\partial\mathbf{x}}|_{\tilde{\mathbf{x}}_k, \mathbf{0}}(\mathbf{x}_k - \tilde{\mathbf{x}}_k) + \frac{\partial h}{\partial \mathbf{v}}|_{\tilde{\mathbf{x}}_k, \mathbf{0}}\mathbf{v}_k
$$

我们令 $\tilde{\mathbf{z}}_k = h(\tilde{\mathbf{x}}_k, 0)$ ， $A = \dfrac{\partial f}{\partial \mathbf{x}}|_{\hat{\mathbf{x}}_{k-1},\mathbf{u}_{k-1}, \mathbf{0}}$ ， $W = \dfrac{\partial f}{\partial \mathbf{w}}|_{\hat{\mathbf{x}}_{k-1},\mathbf{u}_{k-1}, \mathbf{0}}$ ， $H = \dfrac{\partial h}{\partial\mathbf{x}}|_{\tilde{\mathbf{x}}_k, \mathbf{0}}$ ， $V = \dfrac{\partial h}{\partial \mathbf{v}}|_{\tilde{\mathbf{x}}_k, \mathbf{0}}$ ，则有线性化后的模型：

$$\begin{aligned}
\mathbf{x}_k &= \tilde{\mathbf{x}}_k + A(\mathbf{x}_{k-1} - \hat{\mathbf{x}}_{k-1}) + W\mathbf{w}_{k-1}\\
\mathbf{z}_k &= \tilde{\mathbf{z}}_k + H(\mathbf{x}_k - \tilde{\mathbf{x}}_k) + V\mathbf{v}_k
\end{aligned}\tag{6-1}$$

其中，过程噪声和测量噪声的协方差矩阵分别为 $WQ_{k-1}W^\top$ 和 $VR_{k}V^\top$ 。

### 6.1 EKF 公式

**预测过程：**

先验估计：  $\hat{\mathbf{x}}_k^{-} = f(\hat{\mathbf{x}}_{k-1},\mathbf{u}_{k-1},\mathbf{0})$

先验误差协方差矩阵： $P_k^{-} = AP_{k-1}A^\top + WQ_{k-1}W^\top$

**校正过程：**

卡尔曼增益： $K_k = P_k^{-} H^\top \left(HP_k^{-}H^\top + VR_{k}V^\top\right)^{-1}$

后验估计： $\hat{\mathbf{x}}_k = \hat{\mathbf{x}}_k^{-}+K_k\left(\mathbf{z}_k-h\left(\hat{\mathbf{x}}_k^{-},\mathbf{0}\right)\right)$

更新后验误差协方差矩阵：

$$\begin{aligned}
P_k &= (I-K_kH)P_k^{-}(I-K_kH)^{\top} + K_kVR_kV^\top K_k^\top\\
&= (I - K_k H)P_k^{-}
\end{aligned}$$

**需要的初值条件：** $\hat{\mathbf{x}}_0, P_0$ ，即初始状态的后验估计、初始后验估计误差的协方差矩阵。

### 6.2 更常见的情况

一般情况下，我们对噪声对状态转移和测量的影响了解很少；最常见的假设是高斯分布的加性噪声：

$$
\begin{aligned}
\mathbf{x}_k &= f(\mathbf{x}_{k-1},\mathbf{u}_{k-1}) +\mathbf{w}_{k-1}\\
\mathbf{z}_k &= h(\mathbf{x}_k) + \mathbf{v}_k
\end{aligned}
$$

**预测过程：**

先验估计：  $\hat{\mathbf{x}}_k^{-} = f(\hat{\mathbf{x}}_{k-1},\mathbf{u}_{k-1})$

先验误差协方差矩阵： $P_k^{-} = AP_{k-1}A^\top + Q_{k-1}$

**校正过程：**

卡尔曼增益： $K_k = P_k^{-} H^\top \left(HP_k^{-}H^\top + R_{k}\right)^{-1}$

后验估计： $\hat{\mathbf{x}}_k = \hat{\mathbf{x}}_k^{-}+K_k\left(\mathbf{z}_k-h\left(\hat{\mathbf{x}}_k^{-}\right)\right)$

更新后验误差协方差矩阵：

$$\begin{aligned}
P_k &= (I-K_kH) P_k^{-}(I-K_kH)^{\top} + K_kR_k K_k^\top\\
&= (I - K_k H) P_k^{-}
\end{aligned}$$

### 6.3 多条观测量的 EKF 融合滤波

$$
\begin{aligned}
\mathbf{x}_k &= f(\mathbf{x}_{k-1},\mathbf{u}_{k-1}) +\mathbf{w}_{k-1}\\
\mathbf{z}_k^{(1)} &= h_1(\mathbf{x}_k) + \mathbf{v}_k^{(1)}\\
\mathbf{z}_k^{(2)} &= h_2(\mathbf{x}_k) + \mathbf{v}_k^{(2)}\\
&\cdots\\
\mathbf{z}_k^{(j)} &= h_j(\mathbf{x}_k) + \mathbf{v}_k^{(j)}\\
\end{aligned}
$$

其中， $\mathbf{v}^{(i)}_k\sim \mathcal{N}(\mathbf{0},R_k^{(i)})$ .

思路是逐个融合，将上一个融合的结果作为下一次融合的先验。
