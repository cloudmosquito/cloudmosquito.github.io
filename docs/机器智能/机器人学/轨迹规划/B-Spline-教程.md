# B-Spline 曲线：一份零基础也能看懂的教程

> 写给无人机轨迹规划方向的同学 —— 从幼儿园级别开始，一步步搞懂 B-Spline。

!!! 该文件夹目录下有配套的 Python 代码 !!!

---

## 目录

1. [前言：为什么要学 B-Spline？](#1-前言为什么要学-b-spline)
2. [起点：什么是「曲线」？](#2-起点什么是曲线)
3. [线性插值：一切的起点](#3-线性插值一切的起点)
4. [Bézier 曲线：用控制点「拉」出一条曲线](#4-bézier-曲线用控制点拉出一条曲线)
5. [Bézier 的困境：为什么要发明 B-Spline](#5-bézier-的困境为什么要发明-b-spline)
6. [B-Spline 的核心思想：分段多项式](#6-b-spline-的核心思想分段多项式)
7. [基函数：B-Spline 的灵魂](#7-基函数b-spline-的灵魂)
8. [节点向量：决定基函数长什么样](#8-节点向量决定基函数长什么样)
9. [控制点：曲线的「肌肉」](#9-控制点曲线的肌肉)
10. [完整公式与计算实例](#10-完整公式与计算实例)
11. [B-Spline 的核心性质](#11-b-spline-的核心性质)
12. [B-Spline 在无人机轨迹规划中的应用](#12-b-spline-在无人机轨迹规划中的应用)
13. [Python 实战](#13-python-实战)
14. [学习路线建议](#14-学习路线建议)

---

## 1. 前言：为什么要学 B-Spline？

你正在看无人机（UAV）轨迹规划的论文。

你看到这样一段描述：

> *"The trajectory is represented as a uniform B-spline curve of degree 3. The control points serve as optimization variables..."*

你心里想：
- 「样条」是什么？
- 「B 样条」又是什么？B 是什么的缩写？
- 「B-Spline of degree 3」是三个点？三个维度？
- 「控制点」控制什么？
- 「节点向量」又是什么鬼？
- 为什么网上的资料全是 ∑、∏、阶乘、分段定义，看得头疼？

**这份教程就是为你准备的。**

我会假设你**只会初中数学**（一次函数、二次函数），然后一步一步往后推。每一个公式我都会解释「它在干什么」，每一个概念我都会先给直觉再给定义。

### 1.1 「B-Spline」这个名字

- **Spline（样条）**：原意是绘图用的「曲线尺」——一根有弹性的木条/金属条，用几个钉子固定它，它就弯成一条光滑曲线。数学上，「样条」就是分段多项式曲线。
- **B**：**Basis**（基）的缩写。「B-Spline」完整的意思是「基样条」。

所以 B-Spline = **用一套基础函数拼出来的光滑曲线**。

### 1.2 在无人机轨迹规划中，B-Spline 解决了什么？

无人机飞行路径需要满足：
- **光滑**：不能有急转弯（物理上做不到，飞控会炸）
- **可调**：能方便地修改路径（比如避障）
- **高效**：计算快，能实时调整
- **局部修改**：改一小段不影响整条路线

B-Spline 完美满足以上需求。这就是为什么它被大量使用。

---

## 2. 起点：什么是「曲线」？

### 2.1 我们的困境

初中数学里，曲线是 $y = f(x)$ 的形式。比如 $y = x^2$ 是一条抛物线。

但这种方式有一个严重问题：
- **一个 x 只能对应一个 y**
- **无法表示「弯回来」的曲线**

比如你是一个无人机，要从 (0,0) 飞到 (10,5)，中间绕过一个障碍物。你用 $y = f(x)$ 很难描述这条路径，因为途中 x 可能增加也可能减少，一个 x 可能对应多个 y。

### 2.2 解决方案：参数曲线

**换个思路：不把 y 写成 x 的函数，而是把 x 和 y 都写成时间 t 的函数。**

就像描述一个人的行走轨迹——你不需要说「在北纬 X 度时，东经是多少」，而是说「在 t=0 秒时人在哪，t=1 秒时人在哪」。

$$C(t) = \big(x(t),\; y(t)\big)$$

在三维空间就是：

$$C(t) = \big(x(t),\; y(t),\; z(t)\big)$$

参数 t 通常取 [0, 1] 或某个区间。

**这就是参数曲线。** B-Spline 就是一种参数曲线的表示方式。

---

## 3. 线性插值：一切的起点

### 3.1 最简单的情况：两个点之间

你有两个点 $P_0$ 和 $P_1$。你想从 $P_0$ 走到 $P_1$，t 从 0 到 1。

$$C(t) = (1-t) \cdot P_0 + t \cdot P_1$$

- t = 0 时：$C(0) = P_0$
- t = 0.5 时：$C(0.5) = 0.5P_0 + 0.5P_1$（中点）
- t = 1 时：$C(1) = P_1$

这就是**线性插值（Lerp）**。

### 3.2 换个视角：权重函数

把上面的式子改写一下：

$$C(t) = B_0(t) \cdot P_0 + B_1(t) \cdot P_1$$

其中 $B_0(t) = 1-t$，$B_1(t) = t$。

这两个函数 $B_0(t)$ 和 $B_1(t)$ 就是**基函数（Basis Functions）**。

**基函数告诉你：在时刻 t，每个控制点对最终位置的「贡献」有多大。**

- t = 0 时 $B_0(0)=1, B_1(0)=0$，所以位置完全由 $P_0$ 决定
- t = 1 时 $B_0(1)=0, B_1(1)=1$，所以位置完全由 $P_1$ 决定
- t = 0.5 时 $B_0(0.5)=B_1(0.5)=0.5$，各贡献一半

### 3.3 关键觉悟

> **任何参数曲线都可以理解为：基函数 × 控制点，然后求和。**

$$\text{curve}(t) = \sum_{i=0}^{n} B_i(t) \cdot P_i$$

这就是 B-Spline 的核心框架！B-Spline 的特殊之处仅仅在于：**它的基函数长什么样**。

---

## 4. Bézier 曲线：用控制点「拉」出一条曲线

B-Spline 是从 Bézier 曲线发展而来的。理解 Bézier 是理解 B-Spline 的前提。

### 4.1 线性 Bézier（1 阶）

就是上面说的两点线性插值。一阶 Bézier = 直线。

### 4.2 二次 Bézier（2 阶）

你有**三个**控制点 $P_0, P_1, P_2$。

**做法：先对相邻点做线性插值，再对结果做线性插值。**

步骤 1：在 $P_0P_1$ 上取插值点 A，在 $P_1P_2$ 上取插值点 B
$$A(t) = (1-t)P_0 + t P_1$$
$$B(t) = (1-t)P_1 + t P_2$$

步骤 2：在 AB 之间再做一次插值，得到最终点
$$C(t) = (1-t)A(t) + t B(t)$$

代入展开：
$$C(t) = (1-t)^2 P_0 + 2t(1-t) P_1 + t^2 P_2$$

**基函数就是：**
$$B_0(t) = (1-t)^2,\quad B_1(t) = 2t(1-t),\quad B_2(t) = t^2$$

### 4.3 三次 Bézier（3 阶）

四个控制点 $P_0, P_1, P_2, P_3$。

同样的思路——反复做线性插值：

$$C(t) = (1-t)^3 P_0 + 3t(1-t)^2 P_1 + 3t^2(1-t) P_2 + t^3 P_3$$

基函数（称为 **Bernstein 多项式**）：
$$B_0^3(t) = (1-t)^3$$
$$B_1^3(t) = 3t(1-t)^2$$
$$B_2^3(t) = 3t^2(1-t)$$
$$B_3^3(t) = t^3$$

### 4.4 Bézier 的特点

优点：
- ✅ 直观：控制点大致框定了曲线的走向
- ✅ 光滑：曲线是多项式，无限次可导
- ✅ 端点插值：曲线一定经过首尾两个控制点

缺点：
- ❌ **全局性**：动一个控制点，整条曲线都要变（这是致命的）
- ❌ **度数随控制点增加**：10 个控制点就是 9 次多项式，计算量大且数值不稳定
- ❌ **控制力弱**：控制点离曲线较远时，对曲线的「拉扯」力不强

---

## 5. Bézier 的困境：为什么要发明 B-Spline

### 5.1 一个直观的例子

你想用一条曲线经过 20 个航路点。用 Bézier 的话：
- 你需要一条 19 次多项式
- 动其中第 5 个点，整条曲线从头到尾都会变形
- 高次多项式容易出现不想要的振荡（Runge 现象）

这显然不行。你需要的是：
- **局部控制**：动一个点只影响它附近一小段
- **低次多项式**：算得快、数值稳定
- **灵活的段数**：长曲线对应多段，而不是更高次

### 5.2 一个朴素的想法

既然一条 Bézier 曲线搞不定，那**把多条低次 Bézier 曲线拼接起来**行不行？

比如用三段三次 Bézier 曲线首尾相连：
- 第 1 段用 $P_0, P_1, P_2, P_3$
- 第 2 段用 $P_4, P_5, P_6, P_7$
- 第 3 段用 $P_8, P_9, P_{10}, P_{11}$

**问题来了**：拼接处不光滑！两段曲线在连接点的导数不相等，会出现「折角」。

### 5.3 B-Spline 的答案

B-Spline 就是**解决了「如何让多段低次多项式曲线在拼接处自动光滑」这个问题**的方案。

它的做法是：**让相邻的曲线段共享控制点**。

这就像接力赛——每个选手（控制点）只影响自己跑的那一段及其前后一小段，交接棒时速度是连续的。

---

## 6. B-Spline 的核心思想：分段多项式

### 6.1 一句话总结 B-Spline

> **B-Spline 曲线 = 多个低次多项式片段，通过共享控制点的方式自动保证片段间的光滑性。每个控制点只影响它附近的几个片段，而不是整条曲线。**

### 6.2 B-Spline 的三种「零件」

1. **控制点（Control Points）**：$P_0, P_1, ..., P_n$ —— 决定曲线大致形状的「锚点」
2. **节点向量（Knot Vector）**：$t_0, t_1, ..., t_m$ —— 决定「分段」在哪里发生
3. **基函数（Basis Functions）**：$N_{i,k}(t)$ —— 决定每个控制点在参数 t 处的权重

曲线 = 基函数 × 控制点的加权和：

$$C(t) = \sum_{i=0}^{n} N_{i,k}(t) \cdot P_i$$

### 6.3 三个零件之间的关系

一个粗暴但有用的类比：
- **节点向量**是「骨架」，定义了分段的边界
- **基函数**是「肌肉」，定义了每个控制点的势力范围
- **控制点**是「皮肤」，定义了曲线在空间中的实际位置
- **度数 k** 是「光滑程度」，度数越高越光滑（越像橡皮泥），度数越低越「硬」（越像折线）

### 6.4 度数的含义

- k = 1（一阶 B-Spline）：每段是直线，折线。控制点连起来就是曲线本身。
- k = 2（二阶 B-Spline）：每段是二次曲线，在节点处一阶连续（切线连续，无折角）。
- k = 3（三阶 B-Spline，最常用）：每段是三次曲线，在节点处二阶连续（曲率连续，加速度也光滑）。

**无人机的物理约束要求至少 k=3**，因为如果曲率不连续，意味着加速度有突变，飞控会很难受。

---

## 7. 基函数：B-Spline 的灵魂

### 7.1 基函数的直观含义

$N_{i,k}(t)$ 的含义是：**在参数 t 处，第 i 个控制点的「影响力」有多大。**

- $N_{i,k}(t)$ 大 → $P_i$ 使劲拉着曲线往自己这边靠
- $N_{i,k}(t)$ 小 → $P_i$ 的影响很弱
- $N_{i,k}(t) = 0$ → $P_i$ 对 t 处的曲线完全没有影响

所有控制点在任一 t 处的影响力之和恒为 1：$\sum_i N_{i,k}(t) = 1$。

### 7.2 Cox-de Boor 递推：从低阶到高阶

B-Spline 基函数通过一个递推公式定义。这个公式是理解 B-Spline 的核心。

#### 零阶基函数（k=1）：最基础的「开关」

$$N_{i,1}(t) = \begin{cases} 1 & \text{if } t_i \leq t < t_{i+1} \\ 0 & \text{otherwise} \end{cases}$$

**什么意思？** 在每个区间 $[t_i, t_{i+1})$ 上，第 i 个基函数等于 1，出了这个区间就是 0。

就像一个接力棒：一段一段地传，每个时刻只有一个基函数是 1，其余都是 0。

#### 高阶基函数（k ≥ 2）：递推构造

$$N_{i,k}(t) = \frac{t - t_i}{t_{i+k-1} - t_i} \cdot N_{i,k-1}(t) + \frac{t_{i+k} - t}{t_{i+k} - t_{i+1}} \cdot N_{i+1,k-1}(t)$$

**别被吓到！** 这个公式在说什么？

1. $N_{i,k}(t)$ 由两个低一阶的基函数 $N_{i,k-1}(t)$ 和 $N_{i+1,k-1}(t)$ 混合而成
2. 混合比例取决于 t 在两个区间中的相对位置
3. 就像「我既像我爸又像我妈，比例随年龄段不同而变化」

### 7.3 手工计算：从 k=1 到 k=2

让我们用一个具体例子。假设节点向量是 $[0, 0, 1, 2, 3, 3]$（稍后解释为什么这样选）。

用这四个区间 $[0,0), [0,1), [1,2), [2,3), [3,3)$，对应 5 个 k=1 基函数：

```
N_{0,1}(t): 在 [0,0) 上 = 1（这个区间长度为零，没什么用）
N_{1,1}(t): 在 [0,1) 上 = 1
N_{2,1}(t): 在 [1,2) 上 = 1
N_{3,1}(t): 在 [2,3) 上 = 1
N_{4,1}(t): 在 [3,3) 上 = 1（长度为零）
```

现在递推到 k=2：

对于 $N_{1,2}(t)$（使用节点 $t_1=0, t_2=1, t_3=2$）：

$$N_{1,2}(t) = \frac{t - 0}{1 - 0} N_{1,1}(t) + \frac{2 - t}{2 - 1} N_{2,1}(t)$$

- 当 $t \in [0,1)$ 时：$N_{1,1}(t)=1, N_{2,1}(t)=0$ → $N_{1,2}(t) = t$
- 当 $t \in [1,2)$ 时：$N_{1,1}(t)=0, N_{2,1}(t)=1$ → $N_{1,2}(t) = 2-t$
- 当 $t$ 在其他区间时：$N_{1,2}(t) = 0$

**所以 $N_{1,2}(t)$ 是一个三角形的「帐篷」函数！** 

从 0 线性上升到 1，再从 1 线性下降到 0。

k=3 时会变成光滑的钟形函数（三个二次片段拼在一起）。

### 7.4 基函数的关键性质

1. **非负性**：$N_{i,k}(t) \geq 0$ 对所有 t
2. **局部支撑性**：$N_{i,k}(t)$ 仅在 $[t_i, t_{i+k})$ 上非零（出了这个区间，影响力的确就是零）
3. **单位分解**：$\sum_i N_{i,k}(t) = 1$（所有权重加起来是 1）
4. **可微性**：在节点内部无限可微，穿过节点时连续性降低

**性质 2 是理解「局部控制」的关键。** 一个控制点 $P_i$ 的影响范围只有 $[t_i, t_{i+k})$。在这个范围之外，基函数为零，控制点对曲线没有任何影响。

---

## 8. 节点向量：决定基函数长什么样

### 8.1 节点向量是什么

节点向量是一串**非递减**的实数：

$$\mathbf{t} = [t_0, t_1, t_2, ..., t_m]$$

它把参数轴切成很多段，决定了基函数的形状。

**约束条件**：$t_i \leq t_{i+1}$（可以相等，但不能减小）

节点向量的长度 = 控制点数 + 度数 = n + k + 1。

### 8.2 节点间距的意义

节点之间的间距决定了曲线的「速度」：
- 间距大 → 这段曲线「拉得长」，变化平缓
- 间距小 → 这段曲线「压得紧」，变化快

**均匀节点向量**（所有间距相等）：
```
[0, 1, 2, 3, 4, 5, 6] → 间距都是 1
```
简单好用，最常用。

**非均匀节点向量**（间距不等）：
```
[0, 1, 2, 2.5, 3, 5, 7] → 间距为 1, 1, 0.5, 0.5, 2, 2
```
在 [2, 2.5, 3] 这段曲线被「压缩」了，变化剧烈。

### 8.3 重复节点（重节点）

**节点可以重复。** 重复 k 次的节点叫 **k 重节点**。

重要规律：
- 内部节点重复 r 次 → 在该位置，曲线的可微性降低 r-1 次
- 首尾节点重复 k 次 → 曲线会恰好经过首尾控制点（这个非常有用！）

例子：节点向量 `[0, 0, 0, 0, 1, 2, 3, 4, 4, 4, 4]` （k=4）

- 开头四个 0 → 曲线必定经过第一个控制点
- 结尾四个 4 → 曲线必定经过最后一个控制点
- 内部的 1, 2, 3 → 正常的光滑分段

**这是最常用的节点向量类型——Clamped B-Spline**，因为它让曲线起止于首尾控制点，非常方便设计。

### 8.4 三种常见的节点向量类型

| 类型 | 描述 | 用途 |
|------|------|------|
| **Uniform（均匀）** | 所有间隔相等，如 [0,1,2,3,4] | 简单，但一般不经过首尾控制点 |
| **Clamped（夹紧）** | 首尾重节点 k 次 | 最常用！经过首尾控制点 |
| **Open Uniform（开放均匀）** | = Clamped | 同上 |

**在无人机轨迹规划中，Clamped 均匀 B-Spline 是最常见的。** 这样你可以精确控制起点和终点，同时保持中间路径的光滑性。

---

## 9. 控制点：曲线的「肌肉」

### 9.1 控制点的作用

控制点不一定要在曲线上。它们是「吸引」曲线的点。

- 曲线一定位于控制点构成的 **凸包（Convex Hull）** 之内
- 增加控制点 → 曲线更「曲折」，贴合控制多边形更紧
- 减少控制点 → 曲线更「平滑」

### 9.2 局部控制

这是 B-Spline 最重要的性质。

**移动一个控制点 $P_i$，只有参数区间 $[t_i, t_{i+k})$ 上的曲线段会改变。**

为什么？因为基函数 $N_{i,k}(t)$ 在这个区间外为零。

这意味着：
- 一条有 100 个控制点的 B-Spline
- 你移动第 50 个控制点
- 只有第 50 个控制点附近的几段曲线会变
- 远处的曲线纹丝不动

Bézier 曲线做不到这一点。这就是为什么 B-Spline 适合工程设计。

### 9.3 控制点数量、节点数量、度数的关系

对于一个有 n+1 个控制点（编号 0 到 n）、度数为 k 的 B-Spline：

$$m = n + k + 1$$

其中 m+1 是节点向量的长度（节点编号 0 到 m）。

例如：k=3（三次），n+1=7 个控制点 → 节点向量长度 = 7+3+1 = 11 个节点。

---

## 10. 完整公式与计算实例

### 10.1 完整定义

**k 阶 B-Spline 曲线**：

$$C(t) = \sum_{i=0}^{n} N_{i,k}(t) \cdot P_i, \quad t \in [t_{k-1}, t_{n+1}]$$

其中：
- $P_i \in \mathbb{R}^d$ 是控制点（d=2 是平面曲线，d=3 是空间曲线）
- $N_{i,k}(t)$ 是通过 Cox-de Boor 递推定义的基函数
- $\mathbf{t} = [t_0, ..., t_{n+k}]$ 是节点向量

### 10.2 手工计算：一个三次 B-Spline

我们来算一个具体的三次 B-Spline（k=4 阶）。

**参数**：6 个控制点（n=5），k=4，节点向量用 clamped 均匀：
$$\mathbf{t} = [0, 0, 0, 0, 1, 2, 3, 3, 3, 3]$$

> **关于阶与度**：计算机图形学中常用「阶 = 度数 + 1」，所以 k=4 阶对应 3 次。本教程用 k 表示阶。有些文献用 p 表示度（p = k-1）。

节点向量是 10 个（0+1+2+3+4+5+6+7+8+9 = 共 10 个），验证：n+k = 5+4 = 9，节点编号 0~9，共 10 个。✓

曲线定义在 $t \in [t_{k-1}, t_{n+1}] = [t_3, t_6] = [0, 3]$ 上。

**第一个有趣的结果**：当 t=0 时，只有 4 个基函数非零（因为每个基函数的支撑区间长度是 k=4）。
在起始多重节点处，只有 $N_{0,4}(0) = 1$，其余基函数都是 0 → $C(0) = P_0$。✓ 曲线经过第一个控制点。

### 10.3 一般情况下的计算

在实际工程中，计算 B-Spline 的值用这个流程：

1. 给定参数 t，找到 t 所在的节点区间 $[t_j, t_{j+1})$
2. 算出所有在 t 处非零的基函数值（最多 k 个）
3. 用基函数值加权平均对应的控制点

**最常用的算法：de Boor 算法。** 它是 Bézier 曲线的 de Casteljau 算法的推广，就像第 4 节的递推插值一样，但当出现重节点时自动处理分段。

---

## 11. B-Spline 的核心性质

这些性质解释了为什么 B-Spline 在工程中如此受欢迎：

| 性质 | 含义 | 工程意义 |
|------|------|----------|
| **局部控制** | 动一个控制点只影响局部 k 段 | 修改轨迹的一小段不动整体 |
| **凸包性** | 曲线段始终在对应控制点多边形内 | 保证轨迹不会「飞出去」 |
| **可微性** | 内部 r 重节点处为 $C^{k-r-1}$ 连续 | 保证速度和加速度光滑 |
| **变差缩减性** | 任意直线与 B-Spline 的交点数 ≤ 与控制多边形的交点数 | 曲线不会不必要地振荡 |
| **仿射不变性** | B-Spline 经过平移/旋转/缩放后，等于对控制点做同样变换再算曲线 | 变换很方便 |
| **端点插值**（Clamped） | 使用重节点时曲线经过首尾控制点 | 精确指定起点和终点 |

### 11.1 连续性详解

这是轨迹规划最关心的性质。

- **G0 连续（位置连续）**：曲线不断开，k=1 就能保证
- **G1 连续（切线连续）**：没有「折角」，k=2 保证
- **G2 连续（曲率连续）**：转弯是光滑的，加速度连续，k=3 保证
- **G3 连续**：加加速度（jerk）连续，k=4 保证

**对于无人机：**
- k=3（三次 B-Spline）：保证位置、速度、加速度都连续 → 飞控的最基本要求
- k=4（四次 B-Spline）：还保证 jerk 连续 → 更平滑，但计算量更大
- 实际应用中 k=3 最普遍，是性能和光滑度的最佳平衡

---

## 12. B-Spline 在无人机轨迹规划中的应用

### 12.1 为什么选 B-Spline

| 需求 | B-Spline 如何满足 |
|------|-------------------|
| 轨迹光滑（加速度连续） | k ≥ 3 自动保证 $C^2$ 连续 |
| 快速计算 | 低次多项式，de Boor 算法 O(k²) |
| 局部修改（避障） | 局部控制性，只动附近的控制点 |
| 动力学约束 | 凸包性帮助限制速度和加速度边界 |
| 优化变量少 | n 个控制点就能表示复杂轨迹 |

### 12.2 典型的轨迹规划流程

```
1. 确定起点和目标点
   ↓
2. 用路径规划算法（A*、RRT* 等）生成离散航路点
   ↓
3. 用 B-Spline 拟合这些航路点 → 得到初始控制点
   ↓
4. 将控制点作为优化变量，建立优化问题：
    目标：轨迹平滑 + 偏离航路点尽可能小
    约束：速度 ≤ v_max，加速度 ≤ a_max，避开障碍物
   ↓
5. 求解优化 → 得到最优控制点
   ↓
6. 用 B-Spline 公式生成最终轨迹，发给飞控
```

### 12.3 均匀 B-Spline 的特殊便利性

当使用均匀节点向量时，**所有基函数形状相同，只是平移**。这意味着：

- 基函数可以预先算好，拿来就用
- 轨迹的导数（速度、加速度）有闭式表达式
- 计算极快，适合在线重规划

对 k=3 均匀 B-Spline，速度（一阶导）还是 B-Spline（k=2），加速度（二阶导）也是 B-Spline（k=1）。

### 12.4 论文里常见的变体

- **Non-uniform B-Spline (NUBS)**：节点间距不等，能适应不同区域的不同精度需求
- **NURBS**：加了一个权重 w 给每个控制点，能精确表示圆锥曲线（圆、椭圆）
- **B-Spline 优化轨迹**：用 B-Spline 参数化轨迹，控制点作为优化变量

---

## 13. Python 实战

> 以下代码可以直接运行。需要安装：`pip install numpy matplotlib scipy`

### 13.1 手动实现 B-Spline 基函数

```python
import numpy as np
import matplotlib.pyplot as plt


def basis_function(i, k, t, knots):
    """
    Cox-de Boor 递推：计算第 i 个 k 阶 B-Spline 基函数在 t 处的值

    参数:
        i: 基函数索引（从 0 开始）
        k: 阶数（k=1 是分段常数，k=2 是分段线性，k=4 是三次）
        t: 参数值（标量）
        knots: 节点向量（列表或 numpy 数组）
    """
    if k == 1:
        # 零阶基函数：在 [t_i, t_{i+1}) 上为 1，否则为 0
        if knots[i] <= t < knots[i + 1]:
            return 1.0
        else:
            return 0.0

    # 高阶递推
    left = 0.0
    right = 0.0

    # 左项：((t - t_i) / (t_{i+k-1} - t_i)) * N_{i, k-1}(t)
    denominator_left = knots[i + k - 1] - knots[i]
    if denominator_left > 1e-12:  # 避免除以零
        left = ((t - knots[i]) / denominator_left
                * basis_function(i, k - 1, t, knots))

    # 右项：((t_{i+k} - t) / (t_{i+k} - t_{i+1})) * N_{i+1, k-1}(t)
    denominator_right = knots[i + k] - knots[i + 1]
    if denominator_right > 1e-12:
        right = ((knots[i + k] - t) / denominator_right
                 * basis_function(i + 1, k - 1, t, knots))

    return left + right


def bspline_curve(t, control_points, degree, knots):
    """
    计算 B-Spline 曲线在 t 处的值

    参数:
        t: 参数值
        control_points: 控制点列表 [(x1,y1), (x2,y2), ...]
        degree: 多项式的次数（degree=3 是三次）
        knots: 节点向量
    """
    k = degree + 1  # 阶数 = 次数 + 1
    n = len(control_points) - 1  # 控制点最大索引
    result = np.zeros(len(control_points[0]))  # 2D 或 3D

    for i in range(n + 1):
        N = basis_function(i, k, t, knots)
        result += N * np.array(control_points[i])

    return result
```

### 13.2 生成 clamped 节点向量

```python
def clamped_uniform_knots(n_control, degree):
    """
    生成 clamped 均匀节点向量

    参数:
        n_control: 控制点数量
        degree: 多项式次数（3 = 三次）

    返回:
        节点向量（numpy 数组）
    """
    n = n_control - 1  # 控制点最大索引
    k = degree + 1     # 阶
    m = n + k          # 节点向量最大索引

    knots = np.zeros(m + 1)

    # 内部节点均匀分布
    n_internal = n - degree + 1  # 内部节点区间数
    for i in range(k, n + 1):
        knots[i] = (i - k + 1) / (n_internal + 1)

    # 首尾重节点
    for i in range(n + 1, m + 1):
        knots[i] = 1.0

    return knots
```

### 13.3 画图：基函数长什么样

```python
def plot_basis_functions():
    """可视化不同度数的 B-Spline 基函数"""
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))

    for idx, degree in enumerate([1, 2, 3, 4]):
        ax = axes[idx // 2][idx % 2]
        n_control = 7
        k = degree + 1
        knots = clamped_uniform_knots(n_control, degree)

        t_vals = np.linspace(0, 1, 500)
        for i in range(n_control):
            y = [basis_function(i, k, t, knots) for t in t_vals]
            ax.plot(t_vals, y, label=f'$N_{{{i},{k}}}$')

        ax.set_title(f'degree={degree} (k={k} 阶) 的基函数')
        ax.set_xlabel('t')
        ax.set_ylabel('N(t)')
        ax.legend(fontsize=7, loc='upper right')
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('basis_functions.png', dpi=150)
    plt.show()
    print("已保存：basis_functions.png")
```

### 13.4 画图：B-Spline 曲线

```python
def plot_bspline_curve():
    """画一条三次 B-Spline 曲线，展示控制点和曲线的关系"""
    # 控制点
    control_points = np.array([
        [0, 0],
        [1, 3],
        [2, -1],
        [4, 2],
        [5, 0],
        [7, 3],
        [8, 1]
    ])

    degree = 3  # 三次
    knots = clamped_uniform_knots(len(control_points), degree)

    # 采样曲线
    t_samples = np.linspace(0, 1, 200)
    curve = np.array([bspline_curve(t, control_points, degree, knots)
                       for t in t_samples])

    # 画图
    fig, ax = plt.subplots(figsize=(10, 6))

    # 曲线
    ax.plot(curve[:, 0], curve[:, 1], 'b-', linewidth=2, label='B-Spline 曲线')

    # 控制多边形
    ax.plot(control_points[:, 0], control_points[:, 1],
            'ro--', markersize=8, linewidth=1,
            label='控制多边形', markerfacecolor='red')

    # 控制点标注
    for i, (x, y) in enumerate(control_points):
        ax.annotate(f'$P_{i}$', (x, y), textcoords="offset points",
                    xytext=(8, 8), fontsize=11)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('三次 B-Spline 曲线（Clamped，经过首尾控制点）')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')

    plt.tight_layout()
    plt.savefig('bspline_curve.png', dpi=150)
    plt.show()
    print("已保存：bspline_curve.png")
```

### 13.5 演示局部控制

```python
def plot_local_control():
    """演示 B-Spline 的局部控制性：只动一个控制点"""
    fig, ax = plt.subplots(figsize=(10, 6))

    control_original = np.array([
        [0, 0], [1, 2], [2, -0.5], [3, 1.5],
        [4, 0.5], [5, 2], [6, 0]
    ])
    degree = 3
    knots = clamped_uniform_knots(len(control_original), degree)

    # 原始曲线
    t_samples = np.linspace(0, 1, 300)
    curve_orig = np.array([bspline_curve(t, control_original, degree, knots)
                            for t in t_samples])
    ax.plot(curve_orig[:, 0], curve_orig[:, 1], 'b-', linewidth=2,
            label='原始曲线', alpha=0.6)

    # 修改第 3 个控制点（索引 2），向上移动
    control_modified = control_original.copy()
    control_modified[2] = [2, 2.5]  # 原来是 [2, -0.5]

    curve_mod = np.array([bspline_curve(t, control_modified, degree, knots)
                           for t in t_samples])
    ax.plot(curve_mod[:, 0], curve_mod[:, 1], 'r-', linewidth=2,
            label='修改 $P_2$ 后的曲线')

    # 控制多边形
    ax.plot(control_original[:, 0], control_original[:, 1],
            'bo--', markersize=8, linewidth=1, alpha=0.5, label='原控制多边形')
    ax.plot(control_modified[:, 0], control_modified[:, 1],
            'ro--', markersize=8, linewidth=1, alpha=0.5, label='新控制多边形')

    # 标注修改的点
    ax.annotate('$P_2$ 被移动', control_modified[2],
                textcoords="offset points", xytext=(10, 15),
                fontsize=12, color='red',
                arrowprops=dict(arrowstyle='->', color='red'))

    # 标注未改变的段
    ax.annotate('这一段\n完全没变!', (0.2, 0.3), fontsize=11, color='blue')
    ax.annotate('这一段\n完全没变!', (5, 0.5), fontsize=11, color='blue')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('局部控制：只移动一个控制点，只有局部曲线改变')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')

    plt.tight_layout()
    plt.savefig('local_control.png', dpi=150)
    plt.show()
    print("已保存：local_control.png")
```

### 13.6 完整的轨迹生成示例

```python
def drone_trajectory_example():
    """模拟无人机 3D 轨迹规划：用 B-Spline 生成光滑飞行路径"""
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    # 控制点定义无人机大致路径
    control_points_3d = np.array([
        [0, 0, 0],      # 起点
        [2, 1, 1],      # 绕过一个障碍
        [4, 3, 2],      # 爬升
        [6, 1, 3],      # 下降，转向
        [8, 2, 1.5],    # 继续前进
        [10, 0, 0],     # 终点
    ])

    degree = 3
    knots = clamped_uniform_knots(len(control_points_3d), degree)

    # 生成轨迹点
    t_samples = np.linspace(0, 1, 200)
    trajectory = np.array([
        bspline_curve(t, control_points_3d, degree, knots)
        for t in t_samples
    ])

    # 计算速度和加速度（数值微分）
    dt = 1.0 / 200
    velocity = np.gradient(trajectory, dt, axis=0)
    acceleration = np.gradient(velocity, dt, axis=0)
    speed = np.linalg.norm(velocity, axis=1)

    # 画图
    fig = plt.figure(figsize=(16, 10))

    # 3D 轨迹
    ax1 = fig.add_subplot(2, 2, (1, 2), projection='3d')
    ax1.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2],
             'b-', linewidth=2, label='飞行轨迹')
    ax1.scatter(control_points_3d[:, 0], control_points_3d[:, 1],
                control_points_3d[:, 2], c='red', s=80, label='控制点')
    ax1.plot(control_points_3d[:, 0], control_points_3d[:, 1],
             control_points_3d[:, 2], 'r--', alpha=0.4, label='控制多边形')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D B-Spline 飞行轨迹 (degree=3)')
    ax1.legend()

    # 速度曲线
    ax2 = fig.add_subplot(2, 2, 3)
    ax2.plot(t_samples, speed, 'g-', linewidth=1.5)
    ax2.set_xlabel('参数 t')
    ax2.set_ylabel('速率 (m/s)')
    ax2.set_title('速率曲线（光滑连续）')
    ax2.grid(True, alpha=0.3)

    # 加速度曲线
    ax3 = fig.add_subplot(2, 2, 4)
    acc_mag = np.linalg.norm(acceleration, axis=1)
    ax3.plot(t_samples, acc_mag, 'orange', linewidth=1.5)
    ax3.set_xlabel('参数 t')
    ax3.set_ylabel('加速度大小 (m/s²)')
    ax3.set_title('加速度曲线（连续）')
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('drone_trajectory.png', dpi=150)
    plt.show()
    print("已保存：drone_trajectory.png")
    print()
    print("观察要点：")
    print("1. 轨迹是光滑的（没有折角）")
    print("2. 速度曲线是连续的（C¹ 连续）")
    print("3. 加速度曲线也是连续的（C² 连续）")
    print("4. 控制点大致框定轨迹走向，但不一定在轨迹上")
    print("5. 轨迹精确经过起点和终点")


if __name__ == '__main__':
    plot_basis_functions()
    plot_bspline_curve()
    plot_local_control()
    drone_trajectory_example()
```

---

## 14. 学习路线建议

### 14.1 如果你想深入理解

1. **先跑一遍 Python 代码**，改改控制点、改改度数，**培养直觉**
2. 回去看 Cox-de Boor 递推公式，对着代码看（代码就是公式的直接翻译）
3. 手算一个 k=2 的 B-Spline 的例子（5 分钟的事，极大帮助理解）
4. 阅读 de Boor 算法（高效的数值算法）

### 14.2 如果你想看论文

当你再看到论文里这些词时：
- 「uniform B-spline」→ 均匀节点向量，最简单的情况
- 「clamped B-spline」→ 首尾重节点，经过起点终点
- 「degree-3 B-spline」→ 三次多项式，保证加速度连续
- 「control points as decision variables」→ 把控制点坐标当优化变量
- 「knot insertion」→ 插入新节点，局部细化曲线（不改变曲线形状！）
- 「B-spline trajectory optimization」→ 用 B-Spline 参数化轨迹 + 优化控制点

### 14.3 推荐资料

- **《The NURBS Book》** by Piegl & Tiller —— 经典教材，但比较厚
- **MIT 6.837 Computer Graphics** —— 有 B-Spline 的可视化演示
- **"B-Splines" by Dr. C.-K. Shene** —— 非常清晰的在线教程
- **Python `scipy.interpolate.BSpline`** —— 生产级实现，学会自己写之后就可以用这个

---

## 附录 A：常见疑问速查

**Q: B-Spline 的 B 到底是什么？**
A: Basis（基）。B-Spline = Basis Spline = 基于基函数的样条。

**Q: 阶（order）和度（degree）的区别？**
A: 阶 = 度 + 1。三次 B-Spline 是 4 阶的。大部分论文用度（degree），但 Cox-de Boor 递推用阶（k）。注意不要混淆。

**Q: B-Spline 和 Bézier 是什么关系？**
A: Bézier 是 B-Spline 的特例（当节点向量没有内部节点时，B-Spline 退化为 Bézier）。B-Spline 是 Bézier 的推广。

**Q: 为什么叫「样条」？**
A: 源自造船业用的弹性木条（spline），用钉子（控制点）固定，木条自然弯曲成光滑曲线。B-Spline 是这种物理过程的数学抽象。

**Q: 节点向量的值必须是整数吗？**
A: 不。节点可以是任意实数。重要的是它们的**比例关系**。通常归一化到 [0, 1] 方便使用。

**Q: 我需要记住 Cox-de Boor 公式吗？**
A: 不需要死记。理解它的思想（低阶基函数递推混合出高阶基函数），需要时查书或看代码即可。

**Q: 均匀和非均匀 B-Spline 在实际中怎么选？**
A: 默认用均匀（或 clamped 均匀）。只有当你需要某些段更「紧」、某些段更「松」时才用非均匀。无人机轨迹规划中 90% 的情况是均匀 B-Spline。

---

> **最后的话**：B-Spline 的核心思想极其简单——把一条复杂的曲线分解成多个低次多项式的片段，通过基函数让它们自动光滑地拼接起来。那些复杂的公式只是在描述「怎么拼」。一旦你理解了这个思想，你就不再害怕公式了。

---

*教程编写于 2026-06-29，针对零基础学生。如果你发现任何不清楚的地方，随时追问。*
