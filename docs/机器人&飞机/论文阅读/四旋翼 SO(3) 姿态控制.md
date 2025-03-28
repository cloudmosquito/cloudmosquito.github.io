# 四旋翼 SO(3) 姿态控制

## 0 论文与前言

本次阅读的论文为：[Yun Yu, Shuo Yang, Mingxi Wang, Cheng Li and Zexiang Li, "High performance full attitude control of a quadrotor on SO(3)," _2015 IEEE International Conference on Robotics and Automation (ICRA)_, Seattle, WA, USA, 2015, pp. 1698-1703](./四旋翼%20SO(3)%20姿态控制.assets/论文.pdf).

传统的四旋翼控制方法，用欧拉角来表示四旋翼的姿态，分别对三个欧拉角的角度加以控制，忽略了“旋转”本身的 manifold structure 。

三维空间里的姿态旋转，等同于 $\mathrm{SO}(3)$ 里的一个点移动到另一个点。而 yaw-roll-pitch 欧拉角表示法是对 $\mathrm{SO}(3)$ 的一个局部近似，以前者为基础规划控制出来的轨迹是不光滑的，且有奇异点（万向节死锁）问题。

本篇论文将四旋翼在三维空间里的姿态控制，看成 $\mathrm{SO}(3)$ 中的路径规划和轨迹跟踪问题，以实现 $\mathrm{SO}(3)$ 中的光滑轨迹生成，规避欧拉角表示的奇异点问题。

> *Error measure should correspond to the topology of the error space.*
> 
> （反馈式控制中，）误差度量应与误差空间的拓扑结构相对应。

---

## 1 数学基础

李群 $\mathrm{SO}(3)$ 特殊正交群是三维旋转矩阵的集合：

$$\mathrm{SO}(3) = \{\mathbf{R}\in \mathbb{R}^{3\times3}| \mathbf{R}\mathbf{R}^T = \mathbf{I}, \text{det}(\mathbf{R}) = 1\}$$

其对应的李代数 $\mathfrak{so}(3)$ 可以看成等轴角表示下，角速度向量对应的反对称矩阵之集合：

$$\mathfrak{so}(3) = \{\mathbf{S} \in \mathbb{R}^{3\times 3} | \mathbf{S}^T = -\mathbf{S}\}$$

两者之间通过指数映射和对数映射互相对应。考虑一个旋转矩阵 $\mathbf{R} \in \mathrm{SO}(3)$ 和其对应的等轴角表示下的旋转向量 $\hat{\omega}\theta \in \mathfrak{so}(3)$ 。这里的 $\omega$ 是一个单位长度的向量，表征转轴方向；$\hat{\omega}$ 是其对应的反对称矩阵； $\theta$ 表示绕着转轴旋转的角度。

指数映射 $\exp:\mathfrak{so}(3) \to \mathrm{SO}(3)$

$$e^{\hat{\omega}\theta} = \mathbf{I} + \hat\omega\sin\theta + \hat{\omega}^2(1-\cos\theta)$$

对数映射 $\ln:\mathrm{SO}(3) \to \mathfrak{so}(3)$

$$\ln(\mathbf{R}) = \frac{\theta}{2\sin\theta}(\mathbf{R}-\mathbf{R}^T)\ \ \ \ \ \theta = \arccos\frac{\text{tr}(\mathbf{R})-1}{2}$$

---

对于李群 $\mathrm{SO}(3)$ 和其对应的李代数 $\mathfrak{so}(3)$，我们介绍一些性质。

对于任意 $g \in \mathrm{SO}(3)$ 和 $X, Y \in \mathfrak{so}(3)$，我们定义伴随表示 $Ad_{g}$ 和李括号 $ad_X$ ：

$$\begin{aligned}
Ad_g(Y) &= gYg^{-1}\\
ad_{X}(Y) &= \left[X, Y\right] = XY-YX
\end{aligned}$$

在 $\mathrm{SO}(3)$ 上，如下定义一个 $\mathfrak{so}(3) \times \mathfrak{so}(3) \to \mathbb{R}$ 的双线性算符，称之为 “Killing 形式” ：

$$\left<X,Y\right>_K \ \overset{\triangle}= \ \text{tr}(XY)  $$

> 双线性：描述一个映射 $f(x,y)$，满足 $f(ax_1+bx_2, y) = af(x_1, y) + b(x_2, y)$ 和 $f(x, ay_1+by_2) = af(x,y_1) + bf(x,y_2)$ 。

我们可以用 Killing 形式定义 $\mathfrak{so}(3)$ 上的内积：

$$\left<X,Y\right>\ \overset{\triangle}=\ -\frac{1}{4}\left<X,Y\right>_K = -\frac{1}{4}\text{tr}(XY)$$

该内积有以下两条性质：

$$\begin{aligned}
\left<X,Y\right> &= \left<Ad_gX, Ad_gY\right> & \forall g \in \mathrm{SO}(3)\\
\left<ad_ZX,Y\right> &= -\left<X,ad_ZY\right> & \forall Z\in\mathfrak{so}(3)
\end{aligned}$$

给定一个李群 $G$，对于李群中的一个元素 $g$，我们可以用其李代数内积来定义它到单位元（Identity）的距离：

$$\left\|g\right\|_G = \left<\ln(g), \ln(g)\right>^{\frac{1}{2}}$$

我们看一下对于 $\mathrm{SO}(3)$ 而言，这个距离的表达式是什么：

$$\begin{aligned}
\left\|g\right\|_G &= \left<\ln(g), \ln(g)\right>^{\frac{1}{2}}\\
&= \left<\hat{\omega}\theta, \hat{\omega}\theta\right>^{\frac{1}{2}}\\
&= \left(-\frac{1}{4}\theta^2\text{tr}(\hat{\omega}^2)\right)^{\frac{1}{2}}\\
&= \left(\frac{1}{2}\theta^2\right)^{\frac{1}{2}}
\end{aligned}$$

从而我们可以计算距离的导数：

$$\frac{1}{2}\frac{\mathrm{d}}{\mathrm{d}t}\left\|g\right\|_G^2 = \left<\ln(g),V^b\right> = \left<\ln(g),V^s\right>$$

> TODO：这个结论不知道怎么推导出来。

这里的 $V^b$ 和 $V^s$ 如下定义：

$$\dot{g} = gV^b = V^sg\ \ \ \ \ V^b,V^s \in \mathfrak{so}(3)$$

---

## 2 SO(3) 中的四旋翼姿态控制

我们假设能高频连续地测量四旋翼姿态和角速度。实际上，可以通过机载 IMU 以及对应的 IMU 姿态解算算法来实时获得这两个量。

### 2.1 建模与动力学模型

![坐标系](四旋翼%20SO(3)%20姿态控制.assets/坐标系.png){width=80%}

不考虑四旋翼的平移，以 “北东地” 坐标系为惯性坐标系，以 "roll-pitch-yaw" 坐标系为联体坐标系，两个坐标系原点一致。$roll$ 轴角平分 1 号旋臂和 4 号旋臂；yaw 轴垂直机身平面指向下方。

定义以下符号：

$R \in \mathrm{SO}(3)$ 描述联体坐标系相对惯性坐标系的姿态；$J \in \mathbb{R}^{3\times 3}$ 是四旋翼在联体坐标系下的惯性张量；$\omega^b \in \mathbb{R}^3$ 是四旋翼在机体坐标系下的角速度向量；$\tau \in \mathbb{R}^3$ 是四旋翼执行器产生的三轴力矩。

我们有

$$\dot{R} = R\hat{\omega}^b \tag{2-1}$$

$$J\dot{\omega}^b + \omega^b \ \times \ J\omega^b  = \tau \tag{2-2}$$

### 2.2 控制律

我们控制的目标，是将四旋翼联体坐标系的姿态，从当前的 $R_c$ 旋转到目标姿态 $R_t$ ，论文指出这是一个跟踪问题。如果我们把目标姿态看作 $\mathrm{SO}(3)$ 里的一个单位元 $I \in \mathrm{SO}(3)$ （该单位元正好对应于稳定悬停姿态），那么 $R_e = R_tR_c^{-1} = R_c^{-1}$ 就是从单位元到当前姿态的一个旋转。从而，该跟踪问题就被转换为了一个 Regulating（或许翻译成“调节”？）问题。（调节控制输入使 $R_e$ 减小至 0）。

根据 (2-1) 和 (2-2) 式，我们给出四旋翼系统的一阶导形式

$$\begin{cases}\dot{R} &= R\hat{\omega}^b\\
J\dot\omega^b &= -\omega^b \ \times \ J\omega^b + \tau\end{cases} \tag{2-3}$$

论文给出了如下定理：

---

**定理 Ⅰ**

考虑一个 $\mathrm{SO}(3)$ 中的系统

$$\begin{cases}\dot g &= g\hat{V}^b\\
\dot{V}^b &= f(g, V^b)+U\end{cases} \tag{2-4}$$

设 $K_p$ 和 $K_d$ 是对称正定增益矩阵，有控制律

$$U = -f(g,V^b) - K_p \ln(g)^{\vee} - K_d V^b$$

只要初始条件 $\text{tr}(g(0)) \neq -1$ 并且 $K_p$ 和 $V^b(0)$ 满足

$$\lambda_{\min}(K_p) > \frac{\left\|V^b(0)\right\|^2}{\pi^2 - \left\|g(0)\right\|^2_{\mathrm{SO}(3)}}$$

则可证明系统从任意初始状态出发，都能指数稳定在状态 $g \in \mathrm{SO}(3)$，其中 $\lambda_{min}(K_p)$ 是 $K_p$ 的最小特征值。

---

回到 (2-3) 系统，我们应用上述定理，选取控制律为

$$\tau = \omega^b \times J\omega^b - K_p \ln(R)^{\vee} - K_d \omega^b \tag{2-5}$$

其中，第二项根据角度误差做反馈控制，第三项根据角速度误差做反馈控制，整体上就是一个 PD 控制。论文指出可以用串级控制方案代替 (2-5) 式方案，利用" $\ln(R)$ 是 $\hat{\omega}^b$ 的期望值"这一信息，优化控制效果。

论文给了一张非常详细的控制框图，但是存在两个问题：1. 信号流组合的时候加减符号反了；2. Dynamics Feedforward 的式子不对，等号右边第一项应该是 $J\dot{\omega}$。

![控制框图](四旋翼%20SO(3)%20姿态控制.assets/控制框图.png)

首先，有期望姿态 $R_{target}$ 和当前反馈的实际姿态 $R_{current}$ ，两者间的旋转误差用 $R_e = R_{target} R_{current}^T$ 表示，该误差对应的角速度为 $\omega_{desired}^b = \ln(R_e)^{\vee}$ 。

要注意这一步，这相当精巧重要，是整篇论文的核心之处。我们用角度环的误差，直接计算得到了角速度的期望。为什么可以这样计算？因为 $\mathfrak{so}(3)$ 是 $\mathrm{SO}(3)$ 的切空间。这一性质保证了我们生成的角度轨迹在 $\mathrm{SO}(3)$ 空间上连续。

在此基础上，我们先做角速度环 PID ：

$$\begin{aligned}
\text{角速度误差：}&\omega_{e}^b = \omega_{desired}^b - \omega^b = \ln(R_{target}R_{current}^T)^{\vee} - \omega^b\\
\text{角加速度期望值：}& \dot{\omega}_d^b = K_{pv}\omega_{e}^b + K_{dv}\frac{\mathrm{d}}{\mathrm{d}t}\omega_{e}^b
\end{aligned}$$

再做角加速度环 PID ：

$$\begin{aligned}
\text{角加速度误差：}&\dot{\omega}_e^b = \dot{\omega}_d^b - \dot{\omega}^b\\
\text{三轴力矩期望值：}& \tau = K_{pa}\dot{\omega}_{e}^b + K_{ia}\int_{0}^t\dot{\omega}_e^b\mathrm{d}t + \omega_{desired}^b\times J\omega_{desired}^b
\end{aligned}$$

这里论文原文写的是 $\omega^b \times J\omega^b$ ，我倾向于认为作者写错了。我们回顾一下上文提出的 **定理 Ⅰ**，该定理指出能通过适当的控制律使得系统指数稳定在一个 $\mathrm{SO}(3)$ 状态，也就是一个旋转矩阵 $R$ 上；控制律中的 $V^b$ 正是该旋转矩阵求导对应的角速度。回到四旋翼系统，我们要控制的旋转矩阵 $R$ 是从当前姿态到目标姿态的一个旋转变换 $R_e$ ，因此对应的 $V^b$ 应该是 $R_e$ 对应的角速度 $\omega_{desired}^b$ 。

按照 **定理 Ⅰ** ，上式中的 $K_{pv}, K_{dv}, K_{pa}, K_{ia}$ 都得是对称正定矩阵。（我猜要求不一定有这么严格，但也没必要将上式展开成 **定理 Ⅰ** 的形式了）。简单一点，不考虑三轴转动的耦合效应，直接取成对角阵，并且保证元素都是正数即可。

---

### 2.3 工程技巧

论文的控制框图上展示了一些工程技巧，我们在这一小节讨论一下。

首先是计算角速度误差时，论文引入了一个自适应增益 $k_p$ ：

$$\omega_{e}^b = \omega_{desired}^b - \omega^b \overset{\text{引入} k_p}\Longrightarrow \omega_{e}^b = k_p\omega_{desired}^b - \omega^b$$

该增益的作用是调整系统响应速度，不会影响控制器的稳定性。因为只要当前姿态与目标姿态不一致，我们就能求得一个不为 0 的期望角速度 $\ln(R_e)^{\vee}$ 。换言之，只要四旋翼角度有误差，控制器就会让它一直转动，逼近目标角度。

我们直接计算得到的期望角速度包含方向和角速率两个信息，其中关键的是方向信息，这保证了角度轨迹在 $\mathrm{SO}(3)$ 上连续；而角速率，也可以理解为系统的响应速度，是一个我们希望可以调整的参数。这就是 $k_p$ 的作用。

其次，我们的控制器最终输出是四个螺旋桨转速指令；受限于电机性能及其内部的控制器性能，螺旋桨的响应具有一定延迟，这不利于高机动工况下的控制。我们可以考虑将整个四旋翼姿态视为一个具有纯滞后的广义对象，输入是三轴力矩指令，反馈是三轴加速度，采用 Smith 预估器优化控制器性能。

论文中给出的 Pitch 角度广义对象模型为

$$G_p(s) = \frac{e^{-\tau s}}{T_{\phi}s + 1}$$

这里的 $e^{-\tau s}$ 代表对象有一个纯滞后时间 $\tau$ 。除开纯滞后，系统是一个一阶对象，它对于阶跃输入 $u(t) = r$ 的响应是 $y(t) = r\left(1-e^{-\frac{1}{T_{\phi}}t}\right)$ 。所以这里的 $T_{\phi}$ 就代表系统响应到达阶跃输入的 $\left(1-e^{-1}\right)$ ，即 $63.2\%$ 所需要的时间。

最后，控制律用到了惯性张量阵 $J$ ，但论文并没有说明如何对该变量做系统辨识。这将是本文遗留的一个工程问题。

#### Smith 预估器

Smith 预估器还挺简单的，就不另开一篇文章讲了，在这里介绍一下。主要参考浙江大学《过程控制》（张建明）课件。

系统的纯滞后会对反馈式控制造成不利影响。究其原因，是因为**实时反馈信号不能及时描述系统对于控制输入信号的动态反应**。

![原始系统](./四旋翼%20SO(3)%20姿态控制.assets/原始系统.png)

Simith 预估器的思想就是尽量消除反馈信号的延时，从而提升控制效果。也就是说，Smith 预估器想要达到的最佳效果如下：

![理想系统](./四旋翼%20SO(3)%20姿态控制.assets/理想系统.png)

原始系统的闭环传递函数为：

$$\frac{Y(s)}{U(s)} = \frac{CG(s)e^{-\tau s}}{CGO(s)e^{-\tau s}+1}$$

理想系统的闭环传递函数为

$$\frac{Y(s)}{U(s)} = \frac{CG(s)e^{-\tau s}}{1+CGO(s)}$$

我们不加证明地给出，使用以下 Smith 预估器，就能使系统成为上述理想系统：

![Smith预估器系统](./四旋翼%20SO(3)%20姿态控制.assets/Smith预估器系统.png)

Smith 预估器设计的难点和关键点，正是如何获取准确的对象模型 $G(s)$ 和观测器模型 $O(s)$ 。

### 2.4 在 SO(3) 上的路径规划

#### 2.4.1 动机

前文，在数学推导上，Roll、Pitch、Yaw 三轴的地位是完全等价的，但是，在物理实际上，四旋翼的三轴作用是不等价的，主要因为 Yaw 轴的转动惯量更大，力矩系数更小。因此，螺旋桨转速稍稍改变就能引起 Roll、Pitch 的大幅变化；但要想改变 Yaw ，则需要大大改变螺旋桨转速。从另一角度看，这意味着三轴的响应速度有明显差异。这正是我们做路径规划的原因。

#### 2.4.2 做法

根据响应速度的快慢，我们将三轴旋转分为两组，Roll 和 Pitch 为一组，其作用合成为 Tilt；Yaw 为第二组，其作用称为 Torsion 。我们基于此把 $R_e$ 拆分，先 Tilt，再 Torsion ：

$$R_e = R_{torsion}R_{tilt}$$

这个计算也简单。我们规定当前坐标系是 B，Tilt 之后的坐标系是 C，最终的期望坐标系是 T。注意到， $R_{tilt}$ 之后的 $\mathbf{z}_C$ 和最终期望的 $\mathbf{z}_T$ 是一致的，也就是说， $\mathbf{z}_B$ 到 $\mathbf{z}_T$ 完全包含了 Tilt 旋转的信息：

$$R_{tilt} = \exp\left((\mathbf{z}_B \times \mathbf{z}_C)^{\wedge}\right)$$

通过先控制 Tilt ，再控制 Torsion 的方式，我们避开了被控量响应速度不一导致的性能缺陷。

## 3 评价

这篇工作其实工作量不大，提出的核心创新点就是

$$\omega_{desired}^b = \ln(R_{target}R_{current}^T)^{\vee}$$

但是整篇工作关于 $\mathrm{SO}(3)$ 的故事讲得很漂亮，很完整；并且实物抛飞实验也很好地验证了本文算法的有效性，让人印象深刻。
