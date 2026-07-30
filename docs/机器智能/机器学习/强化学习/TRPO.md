# TRPO

## Overview

回顾一下基础策略梯度算法 REINFORCE 的缺陷：

> 1. on-policy 算法，必须让智能体用待更新的策略 $\pi_{\theta}$ 与环境大量交互，采样足够多的轨迹后，才能优化一次策略。
> 2. 对策略提升的学习率（Learning Rate）高度敏感：调得过大，策略更新幅度大，性能可能断崖式下跌；调得过小，每次辛辛苦苦采样所得数据只能优化一点点策略，数据利用率太低。

TRPO 算法的目的是用同一批采样数据做多次优化，并且保证性能不会显著变差，做法是限制策略变化的幅度。

举个例子，智能体最开始以策略 $\pi_1$ 与环境交互，采样得到一批数据 data1。接下来，我们用这批数据优化一次策略，使之更新为 $\pi_2$ 。按照 REINFORCE 算法的理论推导，我们现在不能用 data1 来继续优化策略 $\pi_2$ 。为了解决这一问题，TRPO 算法利用 **重要性采样** 修改目标函数。

**重要性采样** 要求新旧策略的分布不能相差太大。TRPO 算法为保证满足这一条件，直接将其作为约束引入优化问题。

REINFORCE 算法最终可以写成以下优化问题：

$$\max_{\theta}\mathbb{E}_{s\sim\nu^{\pi_{\theta}}(s),a\sim\pi_\theta}\left[\log\pi_{\theta}(a|s)A^{\pi}(s,a)\right]$$

TRPO 算法最终可以写成以下优化问题：

$$\begin{aligned}
\max_{\theta}\ &\mathbb{E}_{s\sim\nu^{\pi_{\theta_{old}}}(s),a\sim\pi_{\theta_{old}}}\left[\frac{\pi_{\theta}(a|s)}{\pi_{\theta_{\rm{old}}}(a|s)}A^{\pi}(s,a)\right]\\
\text{subject to }&\mathbb{E}_{s\sim\nu^{\pi_{\theta_{old}}}}[\mathrm{KL}[\pi_{\theta_{\rm{old}}}(\cdot|s),\pi_{\theta}(\cdot|s)]]\le\delta
\end{aligned}$$

!!! note

    TRPO 都是同策略（on-policy）算法。虽然优化目标中有重要性采样，但因为它们只用到了上一轮策略的数据，并且用于采样的行为策略 $\pi_{\theta_{old}}$ 和待更新的目标策略 $\pi_\theta$ 非常接近，所以它们仍然是同策略算法。


## 理论基础——重要性采样

当 $x$ 服从分布 $p$ 时，若要估计 $f(x)$ 期望值，可从分布 $p$ 中采样一些数据 $x_i$ ，并计算 $f(x_i)$ 的平均值，以近似该期望值：

$$\mathbb{E}_{x\sim p}[f(x)]\approx \frac{1}{N}\sum_{i=1}^{N}f(x_i)$$

现在的问题是，有没有办法不从分布 $p$ 中采样，而是从另一个分布 $q$ 中采样数据，并通过采样所得数据估计 $x$ 服从分布 $p$ 时的 $f(x)$ 期望值？

由于 $\mathbb{E}_{x\sim p}[f(x)] = \int f(x)p(x) \mathrm{d}x$ ，所以可以通过如下变换解决该问题

$$\begin{aligned}
\mathbb{E}_{x\sim p}[f(x)]
&=\int f(x)p(x)\mathrm{d}x\\
&= \int f(x)\frac{p(x)}{q(x)}q(x)\mathrm{d}x\\
&= \mathbb{E}_{x\sim q}\left[f(x)\frac{p(x)}{q(x)}\right]\\
\end{aligned}$$

也就是说，我们从另一个分布 $q$ 中采样数据 $x$ ，计算 $f(x)\dfrac{p(x)}{q(x)}$ 再取期望，即得 $x$ 服从分布 $p$ 时的 $f(x)$ 期望值。这里的 $\dfrac{p(x)}{q(x)}$ 被称为 **重要性权重 (importance weight)** .

### 分布差距不能太大

虽然理论上，我们可以用一个任意的分布 $q$ 来采样数据，估计 $x$ 服从分布 $p$ 时的 $f(x)$ 期望值。但实际中， $p$ 和 $q$ 的差距不能太大。这是为了保证其方差差距不大。

$$\begin{aligned}
\mathrm{Var}_{x\sim p}[f(x)] &= \mathbb{E}_{x\sim p}[f^2(x)] - (\mathbb{E}_{x\sim p}[f(x)])^2\\
\mathrm{Var}_{x\sim q}\left[f(x)\frac{p(x)}{q(x)}\right] &= \mathbb{E}_{x\sim q}\left[\left(f(x)\frac{p(x)}{q(x)}\right)^2\right] - \left(\mathbb{E}_{x\sim q}\left[f(x)\frac{p(x)}{q(x)}\right]\right)^2\\
&= \int f^2(x)\frac{p^2(x)}{q^2(x)}q(x)\mathrm{d}x - \left(\mathbb{E}_{x\sim p}[f(x)]\right)^2\\
&= \int f^2(x)\frac{p(x)}{q(x)}p(x)\mathrm{d}x-(\mathbb{E}_{x\sim p}[f(x)])^2\\
&= \mathbb{E}_{x\sim p}\left[f^2(x)\frac{p(x)}{q(x)}\right]-(\mathbb{E}_{x\sim p}[f(x)])^2
\end{aligned}$$

可见，若 $\dfrac{p(x)}{q(x)}$ 相差很大，则两个随机变量的方差就会相差很大。此时，虽然二者理论上的期望值相同，但是在采样次数不够多的情况下，用一者的采样均值估计另一者的期望值，可能有非常大的误差。

### 重复利用旧数据

在 REINFORCE 算法中，实际每一次更新的梯度可以表示为

$$\mathbb{E}_{s\sim\nu^{\pi_{\theta}}(s),a\sim\pi_\theta}\left[\nabla_{\theta}\log\pi_{\theta}(a|s)A^{\pi}(s,a)\right]$$

现在，我们希望利用 $\pi_{\theta_{old}}$ 采样所得数据，更新策略 $\pi_{\theta}$ ，可以使用重要性采样解决这个问题。

$$\begin{aligned}
&\mathbb{E}_{s\sim\nu^{\pi_{\theta}}(s),a\sim\pi_\theta}\left[\nabla_{\theta}\log\pi_{\theta}(a|s)A^{\pi_{\theta}}(s,a)\right]\\
=& \mathbb{E}_{s\sim\nu^{\pi_{\theta_{old}}}(s),a\sim\pi_{\theta_{old}}}\left[A^{\pi_\theta}(s,a)\nabla_{\theta}\log\pi_{\theta}(a|s) \frac{\pi_{\theta}(s,a)}{\pi_{\theta_{old}}(s,a)}\right]\\
=& \mathbb{E}_{s\sim\nu^{\pi_{\theta_{old}}}(s),a\sim\pi_{\theta_{old}}}\left[A^{\pi_\theta}(s,a)\nabla_{\theta}\log\pi_{\theta}(a|s) \frac{\pi_{\theta}(a|s)p_{\theta}(s)}{\pi_{\theta_{old}}(a|s)p_{\theta_{old}}(s)}\right]
\end{aligned}$$


!!! note

    理论上，此处 $A^{\pi_\theta}(s,a)\nabla_{\theta}\log\pi_{\theta}(a|s)$ 相当于重要性采样中的 $f(x)$ ，所以不会随着分布从 $\pi_{\theta}$ 变为 $\pi_{\theta_{old}}$ 而发生变化。

    但实际上，由于与环境进行交互的是 $\pi_{\theta_{old}}$ ，我们无法估计出 $A^{\pi_\theta}(s, a)$ ，只能估计出 $A^{\pi_{\theta_{old}}}(s,a)$ .

    我们不管三七二十一，直接假设 $A^{\pi_{\theta_{old}}}(s,a) \approx A^{\pi_\theta}(s, a)$ .

    !!! note
    
        上式中的 $p_{\theta}(s)$ 和 $p_{\theta_{old}}(s)$ 在绝大多数情况下难以估计，尤其是 $p_{\theta}(s)$ ，因为 $\pi_\theta$ 都不与环境进行交互。

        同样地，我们不管三七二十一，直接假设 $p_{\theta}(s) \approx p_{\theta_{old}}(s)$ .


从而，异策略更新的梯度可以表示为

$$\mathbb{E}_{s\sim\nu^{\pi_{\theta_{old}}}(s),a\sim\pi_{\theta_{old}}}\left[A^{\pi_{\theta_{old}}}(s,a)\nabla_{\theta}\log\pi_{\theta}(a|s) \frac{\pi_{\theta}(a|s)}{\pi_{\theta_{old}}(a|s)}\right]$$

接下来，利用 $\nabla f(x) = f(x) \nabla \log f(x)$ ，上式可进一步表示为

$$\begin{aligned}
&\mathbb{E}_{s\sim\nu^{\pi_{\theta_{old}}}(s),a\sim\pi_{\theta_{old}}}\left[A^{\pi_{\theta_{old}}}(s,a) \frac{\nabla_{\theta}\pi_{\theta}(a|s)}{\pi_{\theta}(a|s)} \frac{\pi_{\theta}(a|s)}{\pi_{\theta_{old}}(a|s)}\right]\\
=& \nabla_{\theta}\left\{\mathbb{E}_{s\sim\nu^{\pi_{\theta_{old}}}(s),a\sim\pi_{\theta_{old}}}\left[A^{\pi_{\theta_{old}}}(s,a) \frac{\pi_{\theta}(a|s)}{\pi_{\theta_{old}}(a|s)}\right]\right\}
\end{aligned}$$

换言之，当我们使用重要性采样，将策略梯度算法从同策略训练改为"异策略"训练后，我们每次更新策略参数 $\theta$ ，实际上是在优化目标函数

$$J^{\theta_{old}}(\theta) = \mathbb{E}_{s\sim\nu^{\pi_{\theta_{old}}}(s),a\sim\pi_{\theta_{old}}}\left[A^{\pi_{\theta_{old}}}(s,a) \frac{\pi_{\theta}(a|s)}{\pi_{\theta_{old}}(a|s)}\right]$$

> 用 $J^{\theta_{old}}(\theta)$ 表示待优化的目标函数，括号中的 $\theta$ 表示要优化的参数，上标 $\theta_{old}$ 表示与环境做交互，给 $\theta$ 做示范的策略参数。

---

我们来看看这个目标函数到底表达了什么意思。

我们要优化 $\theta$ ，使得对于满足 $\pi_{\theta_{old}}$ 分布的状态动作对 $(s_t,a_t)$ ，$A^{\pi_{\theta_{old}}}(s_t,a_t)\dfrac{\pi_{\theta}(a_t|s_t)}{\pi_{\theta_{old}}(a_t|s_t)}$ 期望值尽可能大。

我们注意到，只有 $\pi_{\theta}(a_t|s_t)$ 一项与要优化的参数 $\theta$ 有关； $A^{\theta_{old}}(s_t,a_t)$ 表示遵循策略 $\pi_{\theta_{old}}$ 时，在状态 $s_t$ 选择动作 $a_t$ 的优势，可正可负； $\pi_{\theta_{old}}(a_t|s_t)$ 表示策略 $\pi_{\theta_{old}}$ 在状态 $s_t$ 选择动作 $a_t$ 的概率，取值在 \[0,1\] 之间。

为了使 $A^{\pi_{\theta_{old}}}(s,a) \dfrac{\pi_{\theta}(a|s)}{\pi_{\theta_{old}}(a|s)}$ 尽可能大，当 $A^{\pi_{\theta_{old}}}(s,a)$ 为正数时，应尽可能使 $\pi_{\theta}(a_t|s_t)$ 增大；当 $A^{\pi_{\theta_{old}}}(s,a)$ 为负数时，应尽可能使 $\pi_{\theta}(a_t|s_t)$ 减小。

也就是说，**当策略 $\pi_{\theta_{old}}$ 通过大量的采样数据，估计在状态 $s_t$ 采取动作 $a_t$ 有优势时，就增加策略 $\pi_\theta$ 在状态 $s_t$ 选择动作 $a_t$ 的概率，反之减小这一概率。**

## 约束新旧策略分布

TRPO 利用重要性采样，将策略梯度算法从同策略方法改为"异策略"方法，使同一批采样数据可以被重复使用，提升了训练效率。

请注意，这里的”重复使用“不是无上限的。”重复使用“的理论基础是”重要性采样“，而”重要性采样“的前提条件是**待更新策略和旧策略的概率分布不能相差太大**，否则，在数据采样量不变的情况下，误差会显著增大。

换一个视角。原始的策略梯度算法，其优化的目标函数为 $J(\theta) = \mathbb{E}_{\tau\sim p(\tau|\theta)}[R(\tau)]$ ，更新方式为 $\theta \leftarrow \theta + \alpha \nabla_{\theta} J(\theta)$ . 这种方法的问题是：

1. 步长过长，新旧策略可能相差太大，导致新策略性能突然显著下降；
2. 步长过小，数据利用率太低。

我们希望有一个新方法能在优化过程中调整步长，保证新策略不要离旧策略太远。

---

为了解决以上问题，TRPO 算法引入 KL 散度约束新旧策略：

$$\mathbb{E}[\mathrm{KL}[\pi_{\theta_{\rm{old}}}(\cdot|s_t),\pi_{\theta}(\cdot|s_t)]]\le\delta$$

同时，TRPO 算法证明了，只要新旧策略分布差距在一定范围内，新策略一定比旧策略更好。这个阈值就是约束式里的 $\delta$ . 当然，理论推导出来的 $\delta$ 值意义不是很大，具体取多少值还得靠测试。

大部分时候其实不用硬约束，而是用惩罚项的形式：

$$\max_{\theta}\ \mathbb{E}_{s\sim\nu^{\pi_{\theta_{old}}}(s),a\sim\pi_{\theta_{old}}}\left[\frac{\pi_{\theta}(a|s)}{\pi_{\theta_{\rm{old}}}(a|s)}A^{\pi}(s,a)-\beta\mathrm{KL}[\pi_{\theta_{\rm{old}}}(\cdot|s),\pi_{\theta}(\cdot|s)]\right]$$

不过这种改进也不是特别好，因为固定的 $\beta$ 不能充分适应优化需求。

作者在 PPO 论文里讨论了 Adaptive KL Penalty Coefficient 方法。一种最简单的方法，先计算 $d = \mathbb{E}[\mathrm{KL}[\pi_{\theta_{old}}(\cdot|s_t),\pi_{\theta}(\cdot|s_t)]]$ ，然后

- if $d < d_{targ}/1.5, \beta \leftarrow \beta/2$
- if $d>d_{targ}\times1.5,\beta \leftarrow \beta\times2$

大量测试证明，这种算法的效果不如 PPO 算法。

> TRPO 是怎么证明的、带 KL 散度约束的优化问题如何处理（需要泰勒展开近似、共轭梯度法、线性搜索等）等问题在理论上都很复杂，我就不学了。
