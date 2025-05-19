# Lyapunov 稳定性

## 0 参考资料

浙江大学控制科学与工程学院《现代控制理论》专业课 PPT（赵豫红老师）。

---

## 1 背景

### 1.1 自治系统

考虑一个系统，有`n`个独立的状态，组成一个**状态变量** $\mathbf{x}_n$，即

$$ \mathbf{x}_n = \begin{bmatrix}x_1 & x_2 &... & x_n\end{bmatrix}^T$$

不妨认为，系统状态都是关于时间`t`的函数，则有

$$ \mathbf{x}_n(t) = \begin{bmatrix}x_1(t) & x_2(t) & ... &x_n(t)\end{bmatrix} $$

如果，系统的状态空间方程可以表示为

$$ \dot{\mathbf{x}}_n(t) = \mathbf{f}(\mathbf{x}_n(t)) $$

那么，我们就称该系统为**自治系统**。这意味着系统不受任何外部输入的影响而变化。

> Note：
>
> 这里的 $\mathbf{f}(\cdot)$ 是一个定义在`n`维空间的向量函数，它可以表示为
>
> $$\mathbf{f} = \begin{bmatrix}f_1 & f_2 & ... & f_n\end{bmatrix}^T$$
>
> 其中，$f_1, f_2, ... ,f_n$ 都是定义在`n`维空间的标量函数。
>
> 举个例子， $\mathbf{f}(\mathbf{x}_2) = \begin{bmatrix}\sin(x_2) & x_1^2\end{bmatrix}^T$ , 那么 $f_1(\mathbf{x}_2) = \sin(x_2)$ ， $f_2(\mathbf{x}_2)=x_1^2$ .

### 1.2 平衡状态/平衡点

如果存在一个状态 $\mathbf{x} = \mathbf{x}_e$，满足 $\mathbf{f}(\mathbf{x_e}) = 0$，则称该状态为 **平衡状态/平衡点**。

> Comment :
>
> 这个定义很好理解，就是该状态下，系统所有状态关于时间的变化率都为0.
>
> 换言之，如果系统初始状态在平衡点，那么系统将始终保持在平衡点。
>
> 但是称其为“稳定点”是不妥当的，“稳定点”应当考虑抗扰性能。譬如 $\dot{\mathbf{x}} = \mathbf{x}$ 和 $\dot{\mathbf{x}}=-\mathbf{x}$ 这两个系统，$\mathbf{x}_e = 0$ 都是平衡点，但是前者稍加扰动就会失稳，而后者具有抵抗一定扰动的能力。我们可以将后者称为稳定点，而前者则不合适。可以说，平衡点是稳定点的必要不充分条件。

#### 1.2.1 线性定常系统的平衡点

考虑我们熟知的线性定常状态空间模型 $\dot{x} = Ax + Bu$，当输入`u`为`0`时，该系统就是一个自治系统

$$ \dot{x} = f(x) = Ax $$

这个系统的平衡点是 `Ax = 0` 这一方程的解。

而根据我们的线性代数知识，我们很容易知道，若 `A` 可逆，则方程只有唯一解 `x = 0`；若`A`不可逆，则方程有无穷多解。通常，我们会将一个不可逆的方阵称为 **奇异的(Singular)**.

---

## 2 Lyapunov 稳定性定义

首先，用 2-范数 定义自治系统关于平衡状态的邻域

$$ S(\mathbf{x}_e, H) = \{\mathbf{x} | \mathbf{x}\in \mathbb{R}^n, \|\mathbf{x}-\mathbf{x}_e\|_2 \le H\} $$

严格定义系统平衡点的 Lyapunov 稳定性：

$$ \forall \varepsilon > 0, \exists \delta > 0, \forall \mathbf{x}(0) \in S(\mathbf{x}_e,\delta), \forall t\in[0,\infty),  \mathbf{x}(t)\in S(\mathbf{x}_e, \varepsilon)$$

![](./Lyapunov%20稳定性.assets/Lyapunov稳定性定义.png){width=50%}

说人话就是，在系统的`n`维状态空间里，以平衡点为球心画个任意大小的球，我都能在其中找到一个更小的球体范围。无论系统状态从新球体中哪一点开始演变，我都能保证系统状态不会超出老球体的范围。

> Note :
>
> 该定义与 $\forall \delta > 0, \exists \varepsilon > 0, \forall \mathbf{x}(0) \in S(\mathbf{x}_e,\delta), \forall t\in [0,\infty),  \mathbf{x}(t)\in S(\mathbf{x}_e, \varepsilon)$ 是不等价的。
>
> 事实上，Lyapunov 稳定性的定义只规定了**系统状态初值在离平衡点无限近的一个邻域内时，系统状态无限趋近于稳定**。而这个新定义则要求系统状态初值在平衡点的任意大邻域内时，系统状态都不发散到无穷。

### 2.1 渐近稳定性

![](./Lyapunov%20稳定性.assets/Lyapunov稳定性分类.png){width=100%}

考虑一个 Lyapunov 稳定的平衡点 $x_e$，如果系统状态从邻域 $S(x_e, \delta)$ 内任意一点开始演变，最后都能收敛到 $x_e$，那么称 $x_e$ 是**渐近稳定的**。把邻域 $S(x_e, \delta)$ 称为 $x_e$ 的**吸引域**。

如果这个自治系统不管初始状态是什么，最后状态都能收敛到 $x_e$，那么称 $x_e$ 是**大范围渐近稳定的**。

平衡点 $x_e$ 在 Lyapunov 意义下**不稳定**的严格表述如下

$$\exists \varepsilon > 0,\forall \delta>0, \exists x(0) \in S(x_e, \delta), \exists t \in[0,\infty), x(t) \notin S(x_e, \varepsilon) $$

---

## 3 如何使用 Lyapunov 稳定

### 3.1 线性定常系统 Lyapunov 稳定的条件

对于单变量线性定常系统 $\dot{x} = Ax, x(0)=x_0, t\ge 0$，有以下定理

**该系统所有的平衡状态都 Lyapunov 稳定的充要条件是 `A`的所有特征值都具有非正实部。**

对于多变量线性定常系统 $\dot{\mathbf{x}} = A\mathbf{x}, \ \mathbf{x}(0) = \mathbf{x}_0, \ t\ge 0$，有以下定理：

**系统的每一平衡状态在 Lyapunov 意义下稳定的充要条件是，$A$ 的所有特征值实部都 $\le 0$，且实部为 0 的特征值为 $A$ 的最小多项式的单根。**

> 方阵的最小多项式：给定一个方阵 A，设其特征方程为 $(\lambda-\lambda_1)^{a_1}(\lambda-\lambda_2)^{a_2}(\cdots)(\lambda-\lambda_s)^{a_s} = 0$ ，那么 A 的一个零化多项式就是 $(A-\lambda_1 I)^{a_1}(A - \lambda_2 I)^{a_2}(\cdots)(A-\lambda_s I)^{a_s}$，A 的最小多项式就是包含因式 $(A-\lambda_1 I)(A-\lambda_2 I)(\cdots)(A-\lambda_s I)$ 的次数最小的零化多项式。
>
> 举个例子，$A = \begin{bmatrix}1 & 0 & 0 & 1 & 0\\ 0 & 2 & 0 & 0 & 0\\ 0 & 0 & 3 & 2 & 0\\ 0 & 0 & 0 & 3 & 0\\ 0 & 0 & 0 & 0 & 2\end{bmatrix}$，其特征值分别为 1，2，3， $(A-I)(A-2I)(A-3I) = \begin{bmatrix}0&0&0&0&0\\0&0&0&0&0\\0 & 0 & 0& 4 & 0\\0&0&0&0&0\\0&0&0&0&0\end{bmatrix} \neq \mathbf{0}$，同时 $(A-I)^2(A-2I)(A-3I)\neq \mathbf{0},(A-I)(A-2I)^2(A-3I)\neq \mathbf{0}$，而 $(A-I)(A-2I)(A-3I)^2 = \mathbf{0}$ ，所以 $A$ 的最小多项式就是 $(A-I)(A-2I)(A-3I)^2$

---

### 3.2 Lyapunov 第一法

**基本思路**：对于一个非线性系统，将它在平衡点位置做线性化，然后按照线性系统的方式来判断其Lyapunov稳定性。

考虑一个非线性自治系统

$$\dot{\mathbf{x}} = \mathbf{f}(\mathbf{x})$$

在 $\mathbf{x} = \mathbf{x}_e$ 位置有一个平衡点，那么我们可以在平衡点位置对其做泰勒展开，得到

$$\dot{\mathbf{x}} = \frac{\partial \mathbf{f}}{\partial \mathbf{x}^T}(\mathbf{x}-\mathbf{x}_e)+O(\mathbf{x})$$

> Comment :
>
> Q：为什么这里求偏导的时候，是对`x`的转置求偏导而不是对`x`本身求偏导？
>
> A：参见[矩阵微积分](../数学/矩阵微积分.md)中关于“分子布局(符号特别版)”的介绍。

这里的 $\frac{\partial \mathbf{f}}{\partial \mathbf{x}^T}$ 本质上是一个 $n\times 1$ 的向量关于一个 $n\times 1$ 的向量求导，结果是

$$\frac{\partial \mathbf{f}}{\partial \mathbf{x}^T} = \begin{bmatrix}\frac{\partial f_1}{\partial x_1} & \frac{\partial f_1}{\partial x_2} & ... & \frac{\partial f_1}{\partial x_n}\\ \frac{\partial f_2}{\partial x_1} & \frac{\partial f_2}{\partial x_2} & ... & \frac{\partial f_2}{\partial x_n}\\ .. & .. &.. & ..\\ \frac{\partial f_n}{\partial x_1} & \frac{\partial f_n}{\partial x_2} & .. & \frac{\partial f_n}{\partial x_n}\end{bmatrix} \overset{\bigtriangleup}{=} A_n$$

我们忽略掉高阶项 $O(\mathbf{x})$，再做变量代换 $\mathbf{z} = \mathbf{x} - \mathbf{x}_e$，即把原来的非线性系统，在平衡点附近化为了如下的线性系统形式

$$\dot{\mathbf{z}} = A_n\mathbf{z}$$

第一法将线性系统的稳定性判据推广到非线性系统，有以下结论：

1. 若线性化后的系数矩阵 $A_n$ 特征值实部全小于 0，则非线性自治系统在 $\mathbf{x}_e$ 处渐近稳定，高阶导数项无影响。
2. 线性化系统的系数矩阵 $A_n$ 只要有一个特征值实部大于 0，则非线性自治系统在 $\mathbf{x}_e$ 处不稳定，高阶导数项无影响。
3. 若线性化系统的系数矩阵 $A_n$ 存在一个特征值实部为 0，其余特征值实部均小于等于 0，则无法用 $A_n$ 判断非线性自治系统在 $\mathbf{x}_e$ 处的稳定性，与高阶导数项相关。

---

### 3.3 Lyapunov 第二法

由于任意一个平衡点都可以通过坐标变换移动到原点，所以我们接下来只考虑平衡点在原点的情况。

> **用能量的观点分析系统的稳定性**：如果系统有一个渐近稳定的平衡点，则当其运动到平衡点的吸引域内时，系统存储能量会逐步衰减至极小值。举个例子，地面上一个半球形的大坑，有小球在坑周边的平地上运动。坑底就是一个小球系统的平衡点。当小球运动到坑的坡面上时，就相当于进入了坑底平衡点的吸引域。当小球逐步滚落到坑底时，小球系统的重力势能逐步衰减。

Lyapunov 构造了一个虚拟的“能量函数” $V(x)$ ，称之为 Lyapunov 函数，用来衡量系统的“能量”，判别平衡点的稳定性。

对于 n 维向量 $\mathbf{x}$ ，$V(\mathbf{x})$ 是一个正定的标量函数。那么 $V(\mathbf{x}) = C$ ，$C$ 为正常数，就描述了 n 维状态空间的一个封闭的超曲面。随着 $\left\|\mathbf{x}\right\|\to \infty$ ，$C \to \infty$ ，该超曲面即扩展为整个状态空间。如果 $C_1 < C_2$ ，则超曲面 1 完全在超曲面 2 内部。

> 沿用上文“大坑”的例子，这个 $V(\mathbf{x}) = C$ 可以形象地理解成等高线。

有几何意义： $V(\mathbf{x})$ 代表状态 $\mathbf{x}$ 到状态空间原点（平衡点）的距离。

---

#### 3.3.1 定理 1

**定理：考虑一个非线性系统 $\dot{\mathbf{x}}(t) = \mathbf{f}(\mathbf{x}(t))$ ，设其平衡状态在原点，即 $\mathbf{f}(\mathbf{0}) = \mathbf{0}$ ，对所有 $t \ge t_0$ ，如果存在一个具有连续一阶偏导数的标量函数 $V(\mathbf{x},t)$ ，满足 $V(\mathbf{x}, t)$ 正定，$\dfrac{\mathrm{d}V}{\mathrm{d}t}(\mathbf{x}, t)$ 负定，则原点处的平衡点是渐近稳定的。**

**进一步地，倘若 $\left\| \mathbf{x} \right\| \to \infty$ 时， $V(\mathbf{x},t) \to \infty$ ，则原点处的平衡点是大范围渐近稳定的。**

> 【思考】这里构造一个正定的 $V(\mathbf{x},t)$ ，其实是定义了一种系统的能量；要求其一阶偏导数负定，要求系统的能量随着时间流逝一直减小，这其实是一个**相当严苛**的要求。
>
> 我感到惊奇的一点是，只需要找到一种能量满足以上条件，就足以说明系统在平衡点处的稳定性。
>
> 仍然举“大坑”的例子，我们假设大坑坡面上有神奇的阻力，能保证小球落到大坑底部时，速度恰好减小至 0 ，并且我们取重力势能的零势能面为大坑底部，这样小球系统的重力势能就是一个正定的标量函数，其导数是负定的。
>
> $\left\| \mathbf{x} \right\| \to \infty$ 时， $V(\mathbf{x},t) \to \infty$ 怎么理解呢？其实就是说这个大坑是无穷大的，“大坑上四周的平地”是不存在的。于是，不管小球初始点在哪里，它都在大坑之中，最终都会滚落到大坑底部。所以大坑底部这个平衡点是大范围渐近稳定的。

说明：

1. Lyapunov 函数不是唯一的
2. 对于渐近稳定的平衡状态，Lyapunov 函数必定存在
3. 用 Lyapunov 第二法证明了系统在某个稳定域内渐近稳定，不代表稳定域外该系统就不稳定了；对于线性系统，如果有渐近稳定的平衡状态，那它必然是大范围渐近稳定的。

---

#### 3.3.2 定理 2

**定理：（克拉索夫斯基，巴巴辛）考虑一个非线性系统 $\dot{\mathbf{x}}(t) = \mathbf{f}(\mathbf{x}(t))$ ，设其平衡状态在原点，即 $\mathbf{f}(\mathbf{0}) = \mathbf{0}$ ，对所有 $t \ge t_0$ ，如果存在一个具有连续一阶偏导数的标量函数 $V(\mathbf{x},t)$ ，满足 $V(\mathbf{x}, t)$ 正定，$\dfrac{\mathrm{d}V}{\mathrm{d}t}(\mathbf{x}, t)$ 半负定； $\dfrac{\mathrm{d}V}{\mathrm{d}t}(\Phi(t;x_0,t_0),t)$ 对于任意 $t_0$ 和任意 $x_0 \neq 0$ ，在 $t \ge t_0$ 时，不恒等于 0 ；当 $\left\|\mathbf{x}\right\| \to \infty$ 时， $V(\mathbf{x}) \to \infty$ ；则原点处的平衡点是大范围渐近稳定的。**

其中的 $\Phi(t;x_0,t_0)$ 表示一条从 $(x_0, t_0)$ 出发的特定轨迹。

> 【思考】这个定理其实挺直观的。它放宽了对于 $\frac{\mathrm{d}V}{\mathrm{d}t}(\mathbf{x},t)$ 的限制，只要求半负定即可，相当于大坑的坡面上有一些位置可以供小球歇脚。但显然小球不能一直歇下去，所以该定理又打了个补丁，要求任意时刻，从坡面上的任意一点出发，小球都不能一直停在坡面某个位置上不往下落。有了这个补丁，大范围渐近稳定的结论还是挺 trival 的。

---

#### 3.3.3 定理 3

**定理：（李雅普诺夫）考虑一个非线性系统 $\dot{\mathbf{x}}(t) = \mathbf{f}(\mathbf{x}(t))$ ，设其平衡状态在原点，即 $\mathbf{f}(\mathbf{0}) = \mathbf{0}$ ，对所有 $t \ge t_0$ ，如果存在一个具有连续一阶偏导数的标量函数 $V(\mathbf{x},t)$ ，满足 $V(\mathbf{x}, t)$ 正定，$\dfrac{\mathrm{d}V}{\mathrm{d}t}(\mathbf{x}, t)$ 半负定； $\dfrac{\mathrm{d}V}{\mathrm{d}t}(\Phi(t;x_0,t_0),t)$ 对于任意 $t_0$ 和任意 $x_0 \neq 0$ ，在 $t \ge t_0$ 时，恒等于 0 ，则系统在原点处的平衡状态是 Lyapunov 意义下稳定的。**

> 【思考】：这个定理和 Lyapunov 稳定性本身一样很难形象地理解，可能是我目前的数学直觉还没到这个层次。

---

#### 3.3.4 定理 4

（关于不稳定性）

**定理：（李雅普诺夫）考虑一个非线性系统 $\dot{\mathbf{x}}(t) = \mathbf{f}(\mathbf{x}(t))$ ，设其平衡状态在原点，即 $\mathbf{f}(\mathbf{0}) = \mathbf{0}$ ，对所有 $t \ge t_0$ ，如果存在一个具有连续一阶偏导数的标量函数 $V(\mathbf{x},t)$ ，满足 $V(\mathbf{x}, t)$ 在原点附近的某一邻域正定，$\dfrac{\mathrm{d}V}{\mathrm{d}t}(\mathbf{x}, t)$ 在同样的邻域内是正定的，则系统在原点处的平衡状态是不稳定的。**

要求放宽一点，**若 $\dfrac{\mathrm{d}V}{\mathrm{d}t}(\mathbf{x},t)$ 半正定，只要 $\mathbf{x} \neq \mathbf{0}$ 时， $\dfrac{\mathrm{d}V}{\mathrm{d}t}(\mathbf{x},t)$ 不恒为零，则系统在原点处的平衡状态依然是不稳定的。**

---

## 4 线性系统

线性系统是非线性系统的一种特殊形式，自然可以用 Lyapunov 第二法进行稳定性分析。

考虑线性自治系统 $\dot{\mathbf{x}} = A\mathbf{x}$ ，这里的 $A$ 是一个非奇异矩阵，所以该系统的平衡状态是唯一的： $\mathbf{x}_e = 0$ . 我们利用 Lyapunov 第二法分析该平衡状态的稳定性。

设 Lyapunov 函数 $V(\mathbf{x}) = \mathbf{x}^T P \mathbf{x}$ ，则其对时间的一阶偏导数为：

$$\begin{aligned}\dot{V}(\mathbf{x}) &= \dot{\mathbf{x}}^TP\mathbf{x} + \mathbf{x}^TP\dot{\mathbf{x}}\\
&= (A\mathbf{x})^TP\mathbf{x}+\mathbf{x}^TP(A\mathbf{x})\\
&=\mathbf{x}^T(A^TP+PA)\mathbf{x}\\
&= -\mathbf{x}^TQ\mathbf{x}\end{aligned}$$

根据 [Lyapunov 第二法定理 1](#331-1)，如果我们能找到正定矩阵 $P,Q$ 满足

$$A^TP+PA+Q = 0 \tag{4-1}$$

则可说明该系统在原点平衡状态渐近稳定。式 (4-1) 被称为 **Lyapunov 方程**。

[根据 Lyapunov 第二法定理 2](#332-2)，如果我们能找到正定矩阵 $P$ 和半正定矩阵 $Q$ ，并且保证沿系统任意一条不恒为零状态的轨迹， $-\mathbf{x}^TQ\mathbf{x}$ 不恒为 0，同样可以判别系统的渐近稳定性。

> 线性定常系统 $\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u}$ 内稳定，等价于对应自治系统 $\dot{\mathbf{x}} = A\mathbf{x}$ 在原点平衡状态渐近稳定。
>
> 【TODO】怎么证明？

需要说明的是，只要选择的矩阵 $Q$ 是（半）正定的，则最终判定结果与 $Q$ 的具体选择无关。

实际应用中一般采取如下方法：取 $Q = I$ （如果能证明沿系统任意一条不恒为零状态的轨迹， $-\mathbf{x}^TQ\mathbf{x}$ 不恒为 0，也可以把 $Q$ 的对角线上挖几个 0 ——半正定），利用 Lyapunov 方程计算 $P$ ，判别 $P$ 的正定性，若正定，则系统渐近稳定。

---

### 4.1 连续系统

**线性定常连续系统 $\dot{\mathbf{x}} = A\mathbf{x}+B\mathbf{u}$ 渐近稳定的充要条件是：给定任意正定实对称矩阵  $Q$ ，存在正定实对称矩阵 $P$ ，满足 Lyapunov 方程：**

$$A^TP+PA+Q=0$$

**若沿系统任意一条不恒为零状态的轨迹， $-\mathbf{x}^TQ\mathbf{x}$ 都不恒等于 0，则可将 $Q$ 的要求放宽至半正定矩阵。**

这里，标量函数 $V(\mathbf{x}) = \mathbf{x}^TP\mathbf{x}$ 是系统的一个 Lyapunov 函数。如果 $P$ 不是正定的而是负定的，则系统不稳定； $P$ 不定，则系统非渐近稳定。

与上一节的讨论相比，该定理对 $P,Q$ 的对称性给出了要求，但我们不应认为这个要求是更高的，事实上，对称和不对称是等价的：

在[二次型与(半)正负定性](../数学/高等代数/二次型与(半)正负定性.md)一文中我们指出， $\mathbf{x}^TP\mathbf{x} = \mathbf{x}^T\dfrac{P+P^T}{2}\mathbf{x}$ ，因此我们可以用 $\dfrac{P+P^T}{2}$ 这一正定实对称矩阵代替正定实矩阵 $P$ ，而不影响上一节的推导过程。因此，可以认为 $P$ 必然是一个实对称矩阵。从而， $Q = -(A^TP+PA)$ 也是一个实对称矩阵。

---

### 4.2 离散系统

我们把上文的结论扩展到离散系统。

线性定常离散系统的状态方程为 $\mathbf{x}(k+1) = G\mathbf{x}(k)$ 。我们取一个 Lyapunov 函数 $V(\mathbf{x}(k)) = \mathbf{x}(k)^TP\mathbf{x}(k)$ ，我们同样利用 [Lyapunov 第二法的定理 1](#331-1) 和 [2](#332-2) 来说明系统的渐近稳定性：

首先，我们需要对 Lyapunov 函数求对时间的一阶偏导数，由于是离散系统，我们采用 $\Delta V(\mathbf{x}) = V(\mathbf{x}(k+1))-V(\mathbf{x}(k))$ 的方式代替求导。

$$\begin{aligned}\Delta V(\mathbf{x}) &= V(\mathbf{x}(k+1))-V(\mathbf{x}(k))\\
&= \mathbf{x}(k+1)^TP\mathbf{x}(k+1) - \mathbf{x}(k)^TP\mathbf{x}(k)\\
&= \mathbf{x}(k)^TG^TPG\mathbf{x}(k)-\mathbf{x}(k)^TP\mathbf{x}(k)\\
&= \mathbf{x}(k)^T(G^TPG-P)\mathbf{x}(k)\\
&= \mathbf{x}(k)^T(-Q)\mathbf{x}(k) \end{aligned}$$

根据 [Lyapunov 第二法定理 1](#331-1) ，系统在平衡状态 $\mathbf{x}_e = \mathbf{0}$ 的充要条件是 $-Q$ 负定；根据  [Lyapunov 第二法定理 2](#332-2) ，如果能保证沿系统任意一条不恒为零状态的轨迹， $\mathbf{x}(k)^TQ\mathbf{x}(k)$ 序列不恒为 0 ，则 $-Q$ 可以放宽要求至半负定。

写成定理：

**线性定常离散系统的状态方程为 $\mathbf{x}(k+1) = G\mathbf{x}(k)$ ，系统在其平衡状态 $\mathbf{x}_e = \mathbf{0}$ 渐近稳定的充要条件是，给定任意一个正定实对称矩阵 $Q$ ，存在一个正定实对称矩阵 $P$ ，满足 Lyapunov 方程**

**$$G^TPG-P+Q=0$$**

**如果能保证沿系统任意一条不恒为零状态的轨迹， $\mathbf{x}(k)^TQ\mathbf{x}(k)$ 序列不恒为 0 ，则 $-Q$ 可以放宽要求至半负定。**

---

## 5 非线性系统

**克拉索夫斯基方法：**

> 该方法本质是考虑二阶导数的情况。

我们考虑非线性系统 $\dot{\mathbf{x}} = f(\mathbf{x})$ ，满足 $f(\mathbf{0}) = \mathbf{0}$ 且 $f(\mathbf{x})$ 对 $x_i$ ， $i=1,2,\cdots,n$ 可微。定义一个雅可比矩阵 $F(\mathbf{x})$

$$F(\mathbf{x}) = \begin{bmatrix}\frac{\partial f_1}{\partial x_1} & \frac{\partial f_1}{\partial x_2} & \cdots & \frac{\partial f_1}{\partial x_n}\\
\frac{\partial f_2}{\partial x_1} & \frac{\partial f_2}{\partial x_2} & \cdots & \frac{\partial f_2}{\partial x_n}\\
\vdots & \vdots & & \vdots\\
\frac{\partial f_n}{\partial x_1} & \frac{\partial f_n}{\partial x_2} & \cdots & \frac{\partial f_n}{\partial x_n}\\
\end{bmatrix}$$

有 $\ddot{\mathbf{x}} = \dot{f}(\mathbf{x}) = F(\mathbf{x})\dot{\mathbf{x}} = F(\mathbf{x})f(\mathbf{x})$ . 我们取 Lyapunov 函数为 $V(\mathbf{x}) = f^T(\mathbf{x})f(\mathbf{x})$ ，并且构造一个 $\hat{F}(\mathbf{x}) = F^T(\mathbf{x})+F(\mathbf{x})$ ，可以证明：**如果 $\forall \mathbf{x}$ ， $\hat{F}(\mathbf{x})$ 是负定的，则平衡状态 $\mathbf{x}_e = 0$ 是渐近稳定的**（**充分条件**）。

首先，我们证明 $V(\mathbf{x}) = f^T(\mathbf{x})f(\mathbf{x})$ 是正定的：

要证明 $V(\mathbf{x}) = f^T(\mathbf{x})f(\mathbf{x})$ 正定，因为已知 $f(\mathbf{0}) = \mathbf{0}$ ，所以只需要证明 $\mathbf{x} \neq \mathbf{0}$ 时， $f(\mathbf{x}) \neq \mathbf{0}$ ，也就是需要证明，非线性系统只有 $\mathbf{x}_e = 0$ 这一个平衡点。

> TODO 我证不出来。

接下来，我们证明 $\dot{V}(\mathbf{x})$ 是负定的：

$$\begin{aligned}\dot{V}(\mathbf{x}) &= \dot{f}^T(\mathbf{x})f(\mathbf{x}) + f^T(\mathbf{x})\dot{f}(\mathbf{x})\\
&= \left[F(\mathbf{x})f(\mathbf{x})\right]^Tf(\mathbf{x}) + f^T(\mathbf{x})F(\mathbf{x})f(\mathbf{x})\\
&=f^T(\mathbf{x})\left[F^T(\mathbf{x}) + F(\mathbf{x})\right]f(\mathbf{x})\\
&= f^T(\mathbf{x})\hat{F}(\mathbf{x})f(\mathbf{x})\end{aligned}$$

因为 $\hat{F}(\mathbf{x})$ 是负定的，所以 $\dot{V}(\mathbf{x})$ 是负定的。
