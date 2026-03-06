我们知道，Q-learning 真正学习的是一个动作价值函数 $Q(s,a)$ 。

如果 s,a 的数量级超大，或者 s,a 干脆是连续的而非离散的，那么我们肯定无法再用一张表格一个个地保存 $Q(s,a)$ 。我们需要一种更加高效的函数拟合方法。DQN 选择的是神经网络，即用 Q-Network 来表示动作价值函数。

## DQN

深度 Q 网络算法（Deep Q-Network）用神经网络表示动作价值函数 $Q(s,a)$ ，从而允许状态 $s$ 是连续的。我们将用于拟合函数 Q 的神经网络称为 Q 网络，用 $Q_{\omega}(s,a)$ 表示（这里的 $\omega$ 表示神经网络参数）。

!!! Note

    一般 DQN（以及 Q-learning）只能处理动作离散的情况，因为函数 Q 更新过程中有 $\max_{a}$ 的操作。

Q-learning 的更新规则为

$$
Q(s,a) \leftarrow Q(s,a) + \alpha\left[r+\gamma\max_{a'\in\mathcal{A}}Q\left(s',a'\right)-Q(s,a)\right]
$$

采用神经网络表示 $Q(s,a)$ 后，其更新方式变为：利用一组数据，更新 Q 网络使得损失函数最小。

我们看一下要怎么训练这个神经网络。

【我们有什么数据？】一系列 $(s,a) \rightarrow r+ \gamma\max_{a'\in\mathcal{A}}Q\left(s',a'\right)$ 的离散数据。

【我们要训练什么？】训练一个神经网络逼近动作价值函数 $Q(s,a)$ 。

【这是一个什么任务？】这其实是一个有监督学习的任务，用带标签的数据训练映射关系。

【怎么定义损失函数？】按照有监督学习的常规算法，用 MSE（mean square error 均方误差）来定义损失函数：

$$
Loss = \frac{1}{N}\sum_{i=1}^{N}\left[Q_{\omega}(s_i,a_i)-\left(r_i+\gamma\max_{a'}Q_{\omega}\left(s_i',a'\right)\right)\right]^2
$$

### 经验回放

回顾一下交互采样的流程：

$$
S_0^{(i)}\overset{a_0^{(i)}}\longrightarrow r_0^{(i)}, s_1^{(i)}\overset{a_1^{(i)}}\longrightarrow r_1^{(i)}, s_2^{(i)}\overset{a_2^{(i)}}\longrightarrow\cdots\overset{a_{T-1}^{(i)}}\longrightarrow r_{T-1}^{(i)}, s_T^{(i)}
$$

每次采样得到的数据是有关联的，如果我们按顺序使用采样结果作为样本训练神经网络，那么样本是不满足 **独立同分布** 假设的，这会大大影响神经网络的训练结果。

一种简单的方式就是把每次的 $(s,a,r,s')$ 四元组数据都存起来；训练 Q 网络的时候，从中随机采样若干数据进行训练。这一方面使样本满足独立假设，另一方面能够让一个样本被多次使用，提升效率。

### 目标网络

我们采样数据的时候，采集的是 $(s,a) \rightarrow r+ \gamma\max_{a'\in\mathcal{A}}Q\left(s',a'\right)$ ，TD 目标里面就用到了我们正在训练的神经网络 $Q_{\omega}$ 。

Q 网络更新方式为：

$$Q_{\omega}(s,a) \leftarrow r+\gamma\max_{a'\in\mathcal{A}}Q_{\omega}(s',a')$$

Q 网络更新的目标 $r+\gamma\max_{a'\in\mathcal{A}}Q_{\omega}(s',a')$ 随着更新过程而不断变化，会导致训练过程不稳定，放大误差。

!!! EXAMPLE 
    
    假设 Q 网络高估了动作 s1, a1 的价值，这部分高估的价值会传递到 s0, a0 上，而 s0, a0 也会进一步传递这一份高估的价值，如此重复甚至会形成循环。因此，DQN 会有价值高估的问题。

为了解决这个问题，前人的思路是用一套暂时固定的"目标 Q 网络"，将 TD 目标中的 Q 网络固定住，让它不要随着 $Q_\omega$ 网络的训练而变化。

详细说来，就是构造一个目标网络 $Q_{\omega^{-}}(s,a)$ ，每隔 C 步，让该目标网络与 $Q_{\omega}(s,a)$ 同步一次。采样数据时，TD 目标为 $r+\gamma\max_{a'\in\mathcal{A}}Q_{\omega^{-}}(s,a)$ 。

#### Double DQN

回顾目标网络提出的解决方案：

$$
Q_{\omega^{-}}(s,a) \overset{periodcally} \leftarrow Q_{\omega}(s,a) \leftarrow r+\gamma\max_{a'\in\mathcal{A}}Q_{\omega^{-}}(s',a')
$$

其本质上仍然存在原始问题：

$$Q_{\omega^{-}}(s,a) \leftarrow r+\gamma\max_{a'\in\mathcal{A}}Q_{\omega^{-}}(s',a')$$

Double DQN 的提出者注意到 $\max_{a'\in\mathcal{A}}Q_{\omega^{-}}(s',a') = Q_{\omega^{-}}(s', \arg\max_{a'}Q_{\omega^{-}}(s',a'))$

Double DQN 的思路就是把 $Q_{\omega^{-}}\left(s', \arg\max_{a'}Q_{\omega^{-}}(s',a')\right)$ 改成 $Q_{\omega^{-}}\left(s', \arg\max_{a'}Q_{\omega}(s',a')\right)$ ，这样就算 Q 网络的价值估计存在误差，也不会被正反馈放大。

即把下式

$$
Q_{\omega^{-}}(s,a) \overset{periodcally} \leftarrow Q_{\omega}(s,a) \leftarrow r+\gamma Q_{\omega^{-}}(s', \arg\max_{a'}Q_{\omega^{-}}(s',a'))
$$

改为

$$Q_{\omega^{-}}(s,a) \overset{periodcally} \leftarrow Q_{\omega}(s,a) \leftarrow r+\gamma Q_{\omega^{-}}(s', \arg\max_{a'}Q_{\omega}(s',a'))$$

这样就再也不会出现正反馈了。

### Dueling DQN

标准 DQN 学习 $Q(s,a)$ 的时候不会考虑状态和动作的影响关系：在某些状态下，执行什么动作奖励都差不多；在另一些状态下，执行特定动作奖励大得多。DQN 所训练的 $Q(s,a)$ 并不能区分到底是因为当前状态获得了若干奖励，还是因为在当前状态下执行特定动作才获得了若干奖励。

Dueling DQN 的思路就是把状态动作价值函数 $Q(s,a)$ 拆成两部分，一部分是状态价值函数 $V(s)$ ，一部分是优势函数 $A(s,a)$ 。优势函数描述了在当前状态下，某一特定动作相对所有可采取的动作的"优势"有多大。显然，优势函数满足

$$
\sum_{a}A(s,a) = 0
$$

基于以上思路，最直接的想法就是训练两个网络分别表示 $V(s)$ 和 $A(s,a)$ ，然后加起来表示 $Q(s,a)$ ，即 $Q(s,a) = V(s) + A(s,a)$ 。

但这样会出现模型训练不唯一的问题： $V(s)$ 增加一个常数， $A(s,a)$ 同样减小一个常数，得到的 $Q$ 值不变。这就导致了训练的不稳定性（因为损失函数仍然是用 $Q$ 值计算的）。

为了解决这个问题，我们采用的方式是引入一个跟 $A$ 有关的项，打破 $V$ 和 $A$ 的平衡性。

实际编程的时候一般是所有 (s,a) 的价值都减去状态 s 时的动作优势函数平均值。

$$
Q_{\eta,\alpha,\beta}(s,a) = V_{\eta,\alpha}(s) + A_{\eta,\beta}(s,a) - \frac{1}{|\mathcal{A}|}\sum_{a'}A_{\eta,\beta}(s,a')
$$

这里的 $\eta,\alpha,\beta$ 都表示网络参数。

!!! Note

    实际编程中，一般让 $V$ 和 $A$ 共用一部分隐藏层。

    ```python
    class Qnet(torch.nn.Module):
        ''' 只有一层隐藏层的Q网络 '''
        def __init__(self, state_dim, hidden_dim, action_dim):
            super(Qnet, self).__init__()
            # Dueling DQN 优化
            self.fc1 = torch.nn.Linear(state_dim, hidden_dim) # 共享网络
            self.fc_A = torch.nn.Linear(hidden_dim, action_dim) # 优势函数
            self.fc_V = torch.nn.Linear(hidden_dim, 1) # 状态价值函数

        def forward(self, x):
            A = self.fc_A(F.relu(self.fc1(x)))
            V = self.fc_V(F.relu(self.fc1(x)))
            Q = V + A - A.mean(1).view(-1, 1)
            return Q
    ```

上式的另一个好处是满足

$$
\sum_{a}\left[A(s,a)-\frac{1}{|\mathcal{A}|}\sum_{a'}A(s,a')\right] = 0
$$
