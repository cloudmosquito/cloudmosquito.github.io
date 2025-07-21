Actor-Critic 算法的特点是结合策略梯度算法和时序差分算法。

Actor 指的是策略函数 $\pi_\theta(a|s)$ ，我们需要学习一个策略以获得尽可能高的回报；Critic 指的是状态价值函数 $V^\pi(s)$ ，我们需要尽可能准确地评估当前策略下某状态的价值。

## 回顾策略梯度

$$\theta \leftarrow \theta + \frac{\eta}{N}\sum_{n=1}^{N} \sum_{t=0}^T\left[\left(\sum_{t'=t}^T \gamma^{t'-t} r^{\{n\}}_{t'} - b\right) \nabla_\theta \log \pi_\theta\left(a^{\{n\}}_t|s^{\{n\}}_t\right)\right]$$

我们用 $G$ 表示某一个状态 $s$ 采取某一个动作 $a$ 之后的累积奖励， $G = \sum_{t'=t}^{T}\gamma^{t'-t}r_{t'}^{\{n\}}$ . $G$ 是一个随机变量，因为某一个状态 $s$ 采取某一个动作 $a$ 之后的交互过程存在随机性。

因此，若采样次数较少，$G$ 可能非常不稳定。不同轨迹之间，相同状态-动作对 (s,a) 所对应的 $G$ 可能相差很大，不能有效近似 $Q^{\pi}(s,a)$ ，导致训练过程不稳定。

## Advantage Actor-Critic 算法

### 核心思想

为了解决上述“不稳定”的问题，我们借鉴 value-based 算法的思路，引入神经网络近似动作价值函数 $Q^{\pi}(s,a)$ 和 $V^{\pi}(s,a)$ ，即

$$\sum_{t'=t}^{T}\gamma^{t'-t}r_{t'}^{\{n\}}-b \overset{代替}\Leftarrow Q^{\pi_\theta}\left(s^{\{n\}}_t,a^{\{n\}}_t\right)-V^{\pi_\theta}\left(s^{\{n\}}_t\right)$$

但是训练两个网络，其误差可能叠加，风险不小。我们能不能只训练一个网络呢？其实是可以的。因为 $Q^{\pi_\theta}\left(s^{\{n\}}_t,a^{\{n\}}_t\right) = \mathbb{E}\left[r_t^{\{n\}}+V^{\pi_\theta}\left(s^{\{n\}}_{t+1}\right)\right]$ ，所以我们可以用 $r_t^{\{n\}}+V^{\pi_\theta}\left(s^{\{n\}}_{t+1}\right)$ 近似动作价值函数，进一步将上式改写为

$$\sum_{t'=t}^{T}\gamma^{t'-t}r_{t'}^{\{n\}}-b \overset{代替}\Leftarrow r_t^{\{n\}}+V^{\pi_\theta}\left(s^{\{n\}}_{t+1}\right)-V^{\pi_\theta}\left(s^{\{n\}}_t\right)$$

从而得到策略梯度的更新式：

$$\theta \leftarrow \theta + \frac{\eta}{N}\sum_{n=1}^{N} \sum_{t=0}^T\left[\left(r_t^{\{n\}}+V^{\pi_\theta}\left(s^{\{n\}}_{t+1}\right)-V^{\pi_\theta}\left(s^{\{n\}}_t\right)\right) \nabla_\theta \log \pi_\theta\left(a^{\{n\}}_t|s^{\{n\}}_t\right)\right]$$


!!! Question

    Q：为什么上文中可以直接去掉期望来近似动作价值函数 $Q$ ？为什么不选择估计动作价值函数 $Q$ 而是选择估计状态价值函数 $V$ ？ 为什么不采用类似 Dueling DQN 的方法，训练一个神经网络表示 $Q$ ，在最后几层中选一个分支直接表示优势函数 $A$ ？

    A：因为原始的 asynchronous advantage actor-critic 算法的论文尝试各种方法后发现上文方法效果最好，所以大家都用该方法。

### 算法流程

策略 $\pi_{\theta_k}$ 与环境交互得到数据，利用类似 DQN 的方式估计价值函数 $V^{\pi_\theta}(s)$ ，再基于 $V^{\pi_\theta}(s)$ 更新策略为 $\pi_{\theta_{k+1}}$ ，重复以上过程。

### 优化技巧

#### 共享神经网络层

由于状态价值函数 $V$ 网络（Critic）和策略网络（Actor）的输入都是状态 $s$ ，因此，它们的前几个层是可以共享的。

尤其是输入信息为 low level 的图像像素信息、声音音频信息、文字信息等时。

#### 策略约束

对策略网络输出的分布设置一个约束，保证分布的熵不太小，即希望不同动作被采用的概率相对均衡，使智能体在训练阶段尽可能尝试不同动作，充分探索环境。


