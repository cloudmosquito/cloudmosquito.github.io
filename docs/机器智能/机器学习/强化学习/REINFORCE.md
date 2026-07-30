# REINFORCE

Q-learning 以及 DQN 算法等都是**基于价值**（value-based）的算法。

先与环境交互获得大量数据，用这些数据拟合一个价值函数，再根据此价值函数导出一个策略（一般是贪心的策略）。

还有一支经典算法，**基于策略**（policy-based）的算法，直接显式学习一个策略。

我们用 $\pi_{\theta}:\mathcal{S}\to\mathcal{A}$ 表示智能体的策略：输入某个状态，输出动作的概率分布。其中 $\theta$ 为该策略函数的参数。同时，假设有一个评价函数 $J(\theta)$ 来评价策略的好坏。

我们想做的是找到最优的策略函数参数 $\theta^*$ 。

$$\theta^* = \arg\max_{\theta} J(\theta)$$

我们希望策略函数：

1. 处处可微，方便我们求导、求梯度，对其进行优化；
2. 具有足够的复杂度以表征复杂策略。

因此，我们选择神经网络作为策略函数。具体选择何种形式，则根据任务、输入输出的不同而定。
## 如何评价策略

如何评价一个策略 $\pi$ 的优秀程度？很简单，用**期望回报**。

我们用形式化的语言加以阐述。首先，考虑环境随机初始状态 $s_0$ ，智能体获得奖励 $r(s_0)$ ；智能体遵循策略 $\pi$ 选择了动作 $a_0$ ，使环境状态改变为 $s_1$ ，获得奖励 $r(s_1)$ ；智能体又遵循策略 $\pi$ 选择了动作 $a_1$ ，如此往复，直到环境终止。我们把这一整个采样过程所获得的数据称为一条轨迹，用 $\tau$ 表示：

$$\tau = \{s_0,r(s_0),a_0,s_1,r(s_1),a_1,\cdots,s_{T-1},r(s_{T-1}),a_{T-1},s_T,r(s_T)\}$$

这条轨迹的总回报是：

$$
R(\tau) = \sum_{t=0}^{T}\gamma^{t}r(s_t)
$$

每条轨迹的真实长度可能不一致，为了统一表示，我们让 $T\to\infty$ 。对于单条轨迹而言，只要设超过长度的 $r(s_k) = 0$ 即可。因此，所有轨迹的总回报，都可以表示为：

$$R(\tau) = \sum_{t=0}^{\infty}\gamma^tr(s_t)$$

这条轨迹的期望回报就是

$$\begin{aligned}
\eta(\pi) &= \sum_{\tau}R(\tau)p(\tau)\\
&=\mathbb{E}_{\tau}\left[R(\tau)\right]\\
&=\mathbb{E}_{s_0,a_0,\dots}\left[\sum_{t=0}^{\infty}\gamma^{t}r(s_t)\right],\text{ where}\\
s_0\sim p(s_0),&a_t\sim\pi(a_t|s_t),s_{t+1}\sim P(s_{t+1}|s_t,a_t)
\end{aligned}$$

我们可以让智能体采用策略 $\pi$ 与环境交互采样大量轨迹，用样本轨迹期望回报的均值近似计算期望回报：

$$\eta(\pi)\approx \frac{1}{N}\sum_{n=1}^{N}R\left(\tau^{(n)}\right)$$

上式中的 $n$ 代表轨迹编号。

> 这种方法学名叫 **蒙特卡洛方法/蒙特卡洛模拟（Monte Carlo Method / Monte Carlo Simulation）** 。

## 如何优化策略

现在我们有能力判断一个策略 $\pi_\theta$ 有多好，方法是大量采样数据计算其期望回报 $\eta(\pi_\theta)$ . 

那么接下来的问题是，我们怎么优化参数 $\theta$ 使其变得更好？

非常经典的做法——梯度上升，即沿着回报值上升最快的方向更新参数 $\theta$ ，形式化表示为：

$$\theta \leftarrow \theta + \alpha \nabla_{\theta} \eta(\pi_{\theta})$$

这里的 $\alpha$ 称为学习率（Learning Rate）。

### 期望回报的梯度

我们现在推导期望回报相对于 $\theta$ 的梯度。

$$\begin{aligned}
\nabla_{\theta}\eta_{\theta} 
&= \nabla_{\theta}\left(\sum_{\tau}R(\tau)p(\tau)\right)\\
&= \sum_{\tau}R(\tau)\nabla_{\theta}p(\tau)\\
\end{aligned}$$

需要计算策略 $\pi_\theta$ 在参数 $\theta$ 下采样到轨迹 $\tau$ 的概率 $p(\tau)$ 。

靠采样轨迹 $\tau_1,\tau_2,\dots,\tau_N$ ，来计算某一条轨迹的概率几乎是不可能的。大多数场景下，轨迹重复的可能性极小。 

我们利用马尔可夫决策过程（MDP）的形式，如下计算概率 $p(\tau)$ ：

$$\begin{aligned}
p(\tau) &= p(s_0)\pi_\theta(a_0|s_0)p(s_1|s_0,a_0)\pi_\theta(a_1|s_1)p(s_2|s_1,a_1)\cdots\\
&= p(s_0)\prod_{t=0}^{\infty}\pi_\theta(a_t|s_t)p(s_{t+1}|s_t,a_t)
\end{aligned}$$

这里 $p(\tau)$ 是一个连乘的形式，很难求梯度。数学上经常采用对数来处理连乘符号，使其变为求和符号，从而可以利用某些操作（比如求梯度）的线性性质。

$$\begin{aligned}
\log p(\tau) 
&= \log \left[p(s_0)\prod_{t=0}^{\infty}\pi_\theta(a_t|s_t)p(s_{t+1}|s_t,a_t)\right]\\
&= \log p(s_0) + \sum_{t=0}^{\infty}\log \pi_\theta(a_t|s_t) + \sum_{t=0}^{\infty}\log p(s_{t+1}|s_t,a_t)\\
\nabla_\theta\log p(\tau)
&= 0 + \sum_{t=0}^{\infty}\nabla_{\theta}\log \pi_{\theta}(a_t|s_t) + 0\\
&= \sum_{t=0}^{\infty}\nabla_{\theta}\log \pi_{\theta}(a_t|s_t)
\end{aligned}$$

现在的问题是，如何将 $\nabla p(\tau)$ 和 $\log p(\tau)$ 联系在一起？


!!! note

    $$\begin{aligned}\nabla \log f(x) &= \dfrac{1}{f(x)}\nabla f(x)\\ \nabla f(x) &= f(x)\nabla \log f(x)\end{aligned}$$


根据以上补充式，我们有 $\nabla p(\tau) = p(\tau) \nabla \log p(\tau)$ . 从而得

$$\begin{aligned}
\nabla_{\theta}\eta(\theta) 
&= \sum_{\tau}R(\tau)\nabla_{\theta}p(\tau)\\
&= \sum_{\tau}R(\tau)p(\tau)\nabla_{\theta}\log p(\tau)\\
&= \sum_{\tau}R(\tau)p(\tau)\left[\sum_{t=0}^{\infty}\nabla_{\theta}\log\pi_{\theta}(a_t|s_t)\right]\\
&= \mathbb{E}_{\tau}\left[R(\tau)\sum_{t=0}^{\infty}\nabla_{\theta}\log\pi_{\theta}(a_t|s_t)\right]
\end{aligned}$$

这里的 $R(\tau)$ 对于求和而言是常数项，可以放进去：

$$\begin{aligned}
\nabla_{\theta}\eta(\theta)
&=\mathbb{E}_{\tau}\left[\sum_{t=0}^{\infty}R(\tau)\nabla_{\theta}\log\pi_{\theta}(a_t|s_t)\right]\\
&\approx \frac{1}{N}\sum_{n=1}^{N}\left[\sum_{t=0}^{T}R(\tau^{(n)})\nabla_{\theta}\log\pi_{\theta}(a_t|s_t)\right]
\end{aligned}$$

上式 $N$ 为轨迹数量，$T$ 第 $n$ 条轨迹的长度。 

理论上，到此为止，我们就可以编写程序了——采样 $N$ 条轨迹的数据，计算每条轨迹的总回报，遍历每条轨迹中的每个状态动作对，代入计算梯度，更新策略，如此循环往复直至收敛。


!!! note "如何理解上式？"

    我们写出具体的策略提升方式：

    $$\theta \leftarrow \theta + \frac{\alpha}{N}\sum_{n=1}^{N} \sum_{t=0}^T R\left(\tau^{(n)}\right)\nabla_\theta \log \pi_\theta\left(a^{(n)}_t|s^{(n)}_t\right)$$

    把求和符号都打开：

    $$\theta \leftarrow \theta + \frac{\alpha}{N}R\left(\tau^{(n)}\right)\nabla_\theta \log \pi_\theta\left(a^{(n)}_t|s^{(n)}_t\right)$$

    $\nabla_{\theta} \log \pi_{\theta}(a_t|s_t)$ 代表一个优化方向，该方向能提升策略 $\pi_{\theta}$ 在状态 $s_t$ 选动作 $a_t$ 的概率，并且是在所有方向中，提升最大的。

    $R\left(\tau^{(n)}\right)$ 代表轨迹 $n$ 中所有动作概率 $\pi_{\theta}(a|s)$ 提升的权重。

    具体而言，所谓策略提升，是在修改 $\pi_{\theta}(a|s)$ . 考虑一条轨迹

    $$\tau = \{s_0,r(s_0),a_0,s_1,r(s_1),a_1,\cdots,s_{T-1},r(s_{T-1}),a_{T-1},s_T,r(s_T)\}$$

    我们用最后的总回报 $\dfrac{\alpha}{N}R(\tau)$ 作为提升该轨迹内每个 $\pi_{\theta}(a_t|s_t)$ 的步长，让 $\theta$ 朝着每个 $\pi_{\theta}(a_t|s_t)$ 提升的方向走 $\dfrac{\alpha}{N}R(\tau)$ 距离。

    直观地来看，就是提升能多拿奖励的动作的概率，减小拿负奖励的动作的概率。

---

接下来的推导过程，我们将在保证期望值不变的前提下，简化期望内的式子。


!!! question "为什么要简化式子？"

    Q：不是已经可以编写程序计算了吗？简化式子的意义是什么？

    A：我们用蒙特卡洛方法近似计算，当样本分布的方差较大时，需要更多样本才能得到足够精确的结果。简化式子，本质目的是为了减小样本方差，提升算法效率，使算法收敛得更快更稳。


### Only Reward-to-go

首先，我们把 $R(\tau)$ 拆成 $t$ 时刻之前和之后两个部分：

$$R(\tau)=\sum_{k=0}^{t-1}\gamma^kr(s_k)+\sum_{k=t}^{\infty}\gamma^kr(s_k)$$

于是得

$$\nabla_{\theta}\eta(\theta)=\mathbb{E}_{\tau}\left[\sum_{t=0}^{\infty}\left(\sum_{k=0}^{t-1}\gamma^kr(s_k)\right)\nabla_{\theta}\log\pi_{\theta}(a_t|s_t)+\sum_{t=0}^{\infty}\left(\sum_{k=t}^{\infty}\gamma^kr(s_k)\right)\nabla_{\theta}\log\pi_{\theta}(a_t|s_t)\right]$$

我们先取前半部分分析。先交换期望和求和的顺序，得

$$
\sum_{t=0}^{\infty}\mathbb{E}_{\tau}\left[\left(\sum_{k=0}^{t-1}\gamma^kr(s_k)\right)\nabla_{\theta}\log\pi_{\theta}(a_t|s_t)\right]\\
$$

$\mathbb{E}_{\tau}\left[\left(\sum_{k=0}^{t-1}\gamma^kr(s_k)\right)\nabla_{\theta}\log\pi_{\theta}(a_t|s_t)\right]$ 可以拆分成 $t-1$ 个期望的求和，每个期望的形式都是：

$$\mathbb{E}_{\tau}\left[\gamma^mr(s_m)\nabla_{\theta}\log\pi_{\theta}(a_t|s_t)\right],\ m<t$$

我们展开计算一下这玩意：

$$\begin{aligned}
&\mathbb{E}_{\tau}\left[\gamma^mr(s_m)\nabla_{\theta}\log\pi_{\theta}(a_t|s_t)\right]\\
=&\int\left[\gamma^mr(s_m)\nabla_{\theta}\log\pi_{\theta}(a_t|s_t)\right]p(\tau)\mathrm{d}\tau\\
=& \int\int\dots\int\left[\gamma^mr(s_m)\nabla_{\theta}\log\pi_{\theta}(a_t|s_t)\right]p(s_0)\prod_{t=0}^{\infty}\pi_\theta(a_t|s_t)p(s_{t+1}|s_t,a_t)\mathrm{d}s_0\mathrm{d}a_0\dots\\
=&\int p(s_0)\mathrm{d}s_0\int\pi_{\theta}(a_0|s_0)\mathrm{d}a_0\dots\int\gamma^mr(s_m)p(s_m|s_{m-1},a_{m-1})\mathrm{d}s_m\dots\\
&\int\nabla_{\theta} \log \pi_{\theta}(a_t|s_t)\pi_{\theta}(a_t|s_t)\mathrm{d}a_t\dots
\end{aligned}$$


!!! note "微积分复习"

    $$\begin{aligned}\int\int xy\mathrm{d}x\mathrm{d}y&=\int y\left(\int x\mathrm{d}x\right)\mathrm{d}y\\&=\int y\mathrm{d}y\int x\mathrm{d}x\end{aligned}$$


我们接下来说明，以上乘式中有一项为 0 ：

$$\begin{aligned}  
\mathbb{E}_{a \sim \pi_{\theta}}[\nabla_{\theta} \log \pi_{\theta}(a|s)]
&= \int \pi_{\theta}(a|s) \nabla_{\theta} \log \pi_{\theta}(a|s) \mathrm{d}a\\
&= \int \pi_{\theta}(a|s) \frac{\nabla_{\theta} \pi_{\theta}(a|s)}{\pi_{\theta}(a|s)} \mathrm{d}a\\
&= \int \nabla_{\theta} \pi_{\theta}(a|s) \mathrm{d}a \\  
&= \nabla_{\theta} \int \pi_{\theta}(a|s) \mathrm{d}a \\  
&= \nabla_{\theta} (1) = 0  
\end{aligned}$$

因此，我们上文计算的期望回报左半部分，等于 0 ！所以有

$$\nabla_{\theta}\eta(\theta)=\mathbb{E}_{\tau}\left[\sum_{t=0}^{\infty}\left(\sum_{k=t}^{\infty}\gamma^kr(s_k)\right)\nabla_{\theta}\log\pi_{\theta}(a_t|s_t)\right]$$

回顾 [马尔可夫决策过程](马尔可夫决策过程.md)，可以发现，式中的 $\sum_{k=t}^{\infty}\gamma^kr(s_k)=\gamma^t\sum_{k=t}^{\infty}\gamma^{k-t}r(s_k)=\gamma^t G_t$ . 所以有

$$\nabla_{\theta}\eta(\theta)=\mathbb{E}_{\tau}\left[\sum_{t=0}^{\infty}\gamma^tG_t\nabla_{\theta}\log\pi_{\theta}(a_t|s_t)\right]$$

!!! note "如何理解上式？"

    我们要提升能多拿奖励的动作的概率，减小拿负奖励的动作的概率，方式是用$\dfrac{\alpha}{N}R\left(\tau^{(n)}\right)$ 作为提升动作概率 $\pi_{\theta}\left(a_t^{(n)}|s_t^{(n)}\right)$ 的步长。

    但是，$t$ 时刻之前的奖励，无法用于判断在 $s_t$ 状态采取动作 $a_t$ 拿到的奖励是正是负，是多是少。

    所以，这部分奖励是无意义的。

    删去这部分奖励后，原先的 $R\left(\tau^{(n)}\right)$ 就变为 $G_t^{(n)}$ 。


### 讨厌的轨迹期望

一路推导下来，发现 $\mathbb{E}_{\tau}$ 这玩意实在是令人讨厌，包含一大堆东西，很不直观。我们试试把它干了。首先，交换期望与求和：

$$\nabla_{\theta}\eta(\theta)=\sum_{t=0}^{\infty}\mathbb{E}_{\tau}\left[\gamma^tG_t\nabla_{\theta}\log\pi_{\theta}(a_t|s_t)\right]$$

我们考虑和式中的任意一项，把对轨迹的期望写成对所有变量的积分，并且把轨迹分为过去 $\Gamma=(s_0,a_0,\dots,s_{t-1},a_{t-1})$，当前 $(s_t,a_t)$ 和未来 $\Pi=(s_{t+1},a_{t+1},\dots)$ ，则有

$$
\mathbb{E}_{\tau}\left[\gamma^tG_t\nabla_{\theta}\log\pi_{\theta}(a_t|s_t)\right]=\int\left[\gamma^tG_t\nabla_{\theta}\log\pi_{\theta}(a_t|s_t)\right]p(\Gamma,s_t,a_t,\Pi)\mathrm{d}\Gamma\mathrm{d}s_t\mathrm{d}a_t\mathrm{d}\Pi
$$

其中， $p(\Gamma,s_t,a_t,\Pi)=p(s_t,a_t)p(\Gamma|s_t,a_t)p(\Pi|s_t,a_t,\Gamma)$ 。由于马尔可夫性，未来仅和当前状态动作有关，和过去无关，因此 $p(\Pi|s_t,a_t,\Gamma)=p(\Pi|s_t,a_t)$ 。于是，上式可写成

$$\begin{aligned}
& \gamma^t\int p(\Gamma|s_t,a_t)\mathrm{d}\Gamma\int_s\int_a\nabla_{\theta}\log\pi_{\theta}(a_t|s_t)p(s_t,a_t)\left[\int G_tp(\Pi|s_t,a_t)\mathrm{d}\Pi\right]\mathrm{d}a_t\mathrm{d}s_t\\
=&\gamma^t\times1\times\int_s\int_a\nabla_{\theta}\log\pi_{\theta}(a_t|s_t)p(s_t,a_t)\mathbb{E}[G_t|s_t,a_t]\mathrm{d}a_t\mathrm{d}s_t
\end{aligned}$$

其中，$\mathbb{E}[G_t|s_t,a_t]$ 是动作价值函数 $Q^{\pi}(s_t,a_t)$ ；$p(s_t,a_t)=p(s_t)\pi_{\theta}(a_t|s_t)$ 。于是，原梯度可写成

$$\begin{aligned}\nabla_{\theta}\eta(\theta)
&=\sum_{t=0}^{\infty} \int_s\int_a\gamma^tp(s_t)\nabla_{\theta}\log\pi_{\theta}(a_t|s_t)\pi_{\theta}(a_t|s_t)Q^{\pi}(s_t,a_t)\mathrm{d}a_t\mathrm{d}s_t\\
&=\int_s\sum_{t=0}^{\infty}\gamma^tp(s_t)\left[\int_a\nabla_{\theta}\log\pi_{\theta}(a_t|s_t)\pi_{\theta}(a_t|s_t)Q^{\pi}(s_t,a_t)\mathrm{d}a_t\right]\mathrm{d}s_t\\
&=\frac{1}{1-\gamma} \int_s(1-\gamma)\sum_{t=0}^{\infty}\gamma^tp(s_t)\left[\int_a\nabla_{\theta}\log\pi_{\theta}(a_t|s_t)\pi_{\theta}(a_t|s_t)Q^{\pi}(s_t,a_t)\mathrm{d}a_t\right]\mathrm{d}s_t\\
&= \frac{1}{1-\gamma} \int_s\nu^{\pi_{\theta}}(s)\mathbb{E}_{a\sim\pi_{\theta}}[\nabla_{\theta}\log\pi_{\theta}(a|s_t)Q^{\pi}(s_t,a)]\mathrm{d}s_t\\
\end{aligned}$$

参见 [马尔可夫决策过程](./马尔可夫决策过程.md) ，这里的 $\sum_{t=0}^{\infty}\gamma^tp(s_t)$ 就是一个归一化的状态访问分布 $\nu^{\pi_{\theta}}(s)$ ，描述了智能体采用策略 $\pi_\theta$ 访问到状态 $s$ 的概率分布。因此有

$$\nabla_{\theta}\eta(\theta)=\frac{1}{1-\gamma}\mathbb{E}_{s\sim\nu^{\pi_{\theta}}(s),a\sim\pi_\theta}\left[\nabla_{\theta}\log\pi_{\theta}(a|s)Q^{\pi}(s,a)\right]$$

梯度是一个方向，其数值大小没什么意义，所以我们直接把 $\dfrac{1}{1-\gamma}$ 扔掉。或者说，把它放到 Learning Rate 里面：

$$\alpha\nabla_{\theta}\eta(\theta)=\alpha\mathbb{E}_{s\sim\nu^{\pi_{\theta}}(s),a\sim\pi_\theta}\left[\nabla_{\theta}\log\pi_{\theta}(a|s)Q^{\pi}(s,a)\right]$$

### Add Baseline

上文已经推导过，$\mathbb{E}_{a\sim\pi_\theta}[\nabla_\theta\log\pi_{\theta}(a|s)] = 0$ 。如果我们在期望中乘一个与 $a$ 无关的值，该式依然为 0 。因此，为了减小样本方差，我们可以将期望内的式子改写为：

$$\nabla_{\theta}\eta(\theta)=\mathbb{E}_{s\sim\nu^{\pi_{\theta}}(s),a\sim\pi_\theta}\left\{\nabla_{\theta}\log\pi_{\theta}(a|s)\left[Q^{\pi}(s,a)-b\right]\right\}$$

接下来的问题是， $b$ 取何值的时候，样本方差最小？这个严格推导有点复杂，实在要写的话，亲测 gemini 也能回答，就不写了。

结论是，综合考量最优性和工程可实现性，选择 $b=V^{\pi}(s)$ 。因为优势函数 $A^{\pi}(s,a)=Q^{\pi}(s,a)-V^{\pi}(s)$ ，所以

$$\nabla_{\theta}\eta(\theta)=\mathbb{E}_{s\sim\nu^{\pi_{\theta}}(s),a\sim\pi_\theta}\left[\nabla_{\theta}\log\pi_{\theta}(a|s)A^{\pi}(s,a)\right]$$

!!! note "如何理解上式？"

    如果奖励总是正的，理想情况下，所有动作概率 $\pi_{\theta}(a|s)$ 都会提升，只不过根据所在轨迹的总回报不同，有的动作概率提升得多，有的动作概率提升得少。提升之后，所有动作概率总和大于 1 。这时候我们再做归一化，就完成了一次策略提升。

    但以上都是理想情况。如果我们采样的轨迹不够多，恰好遗漏了一个奖励很大的动作 $a_m$ ，那么在概率提升时，其他动作概率都或多或少提升了，而该动作概率保持不变。归一化之后，该动作的概率反而下降了！

    想解决这个问题也很简单，我们只要将它减去奖励的期望，让奖励均匀分布在 0 周围，有正有负即可


## 小结

最终，REINFORCE 算法的更新策略是：

$$\theta \leftarrow \theta + \alpha\sum_{n=1}^{N} \sum_{t=0}^T\left[\left(\sum_{t'=t}^T \gamma^{t'-t} r^{\{n\}}_{t'} - b\right) \nabla_\theta \log \pi_\theta\left(a^{\{n\}}_t|s^{\{n\}}_t\right)\right]$$

我们总结一下整个算法思路。

目的：优化策略 $\pi_{\theta}$ ，即优化 $\mathbb{E}_{s_0,a_0,\dots}\left[\sum_{t=0}^{\infty}\gamma^{t}r(s_t)\right]$ 。

方法：对 $\mathbb{E}_{s_0,a_0,\dots}\left[\sum_{t=0}^{\infty}\gamma^{t}r(s_t)\right]$ 关于 $\theta$ 求梯度。

具体做法：用当前策略采一批轨迹，用这些数据近似 $A^{\pi}(s,a)$ ，然后代入上式优化策略；然后再用新的策略去采新数据，如此往复。

缺陷：

1. on-policy 算法，必须让智能体用待更新的策略 $\pi_{\theta}$ 与环境大量交互，采样足够多的轨迹后，才能优化一次策略。
2. 对策略提升的学习率（Learning Rate）高度敏感：调得过大，策略更新幅度大，性能可能断崖式下跌；调得过小，每次辛辛苦苦采样所得数据只能优化一点点策略，数据利用率太低。