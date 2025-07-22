KL 散度（Kullback-Leibler divergence，简称 KLD），又有别称**相对熵** （relative entropy），消息增益（information gain），消息散度（information divergence）等。

## 定义

对于两个定义在同一样本空间 $\mathcal{X}$ 的离散概率分布 $P$ 和 $Q$ ，KL 散度定义为

$$D_{\mathrm{KL}}(P||Q) = \sum_{x\in\mathcal{X}}P(x)\ln\frac{P(x)}{Q(x)}$$

对于连续分布，上式改为积分形式

$$D_{\mathrm{KL}}(P||Q) = \int_{\mathcal{X}}p(x)\ln\frac{p(x)}{q(x)}$$

这里的 $p(x)$ 和 $q(x)$ 分别是分布 $P$ 和 $Q$ 的概率密度函数。约定 $0\ln(0/q) = 0$ 和 $(p/0)\ln(p/0) = +\infty$ .

这里的对数一般是以 $e$ 为底，单位是 nat ；若以 2 为底，单位就是 bit 。

## 性质

### 非负性

KL 散度的非负性等价于 Gibbs 不等式：

设有离散概率分布 $P$ 和 $Q$ ，则有

$$D_{\mathrm{KL}}(P||Q) = \sum_{x\in\mathcal{X}}P(x)\ln\frac{P(x)}{Q(x)} \ge 0$$

当且仅当 $P(x_i) = Q(x_i)$ 对下标 $i=1,2,\cdots,n$ 都成立时，等号成立。

#### 证明

要证 $\sum_{x\in\mathcal{X}}P(x)\ln\dfrac{P(x)}{Q(x)} \ge 0$ ，即证 $-\sum_{x\in\mathcal{X}}P(x)\ln\dfrac{Q(x)}{P(x)} \ge 0$ . 

设使得 $P(x) >0$ 的 $x$ 集合为 $\mathcal{M}$ . 因为 $P(x) =0$ 时，$P(x)\ln\dfrac{P(x)}{Q(x)}  = 0$ ，所以相当于要证明 $\sum_{x\in\mathcal{M}}P(x)\ln\dfrac{Q(x)}{P(x)} \le 0$ .

$$\begin{aligned}
\sum_{x\in\mathcal{M}}P(x)\ln\dfrac{Q(x)}{P(x)} 
&\le \sum_{x\in\mathcal{M}}P(x)\left[\frac{Q(x)}{P(x)}-1\right]\\
&=\sum_{x\in\mathcal{M}}Q(x)-\sum_{x\in\mathcal{M}}P(x)\\
&= \sum_{x\in\mathcal{M}}Q(x) - 1\\
& \le 0
\end{aligned}$$

接下来是等号取等条件。首先， $\sum_{x\in\mathcal{M}}Q(x)-1 \le 0$ 的取等条件是 $\sum_{x\in\mathcal{M}}Q(x) = 1$ ；其次，$\sum_{x\in\mathcal{M}}P(x)\ln\dfrac{Q(x)}{P(x)} \le \sum_{x\in\mathcal{M}}P(x)\left[\dfrac{Q(x)}{P(x)}-1\right]$ 的取等条件是 $\dfrac{Q(x)}{P(x)}=1, x\in\mathcal{M}$ . 当且仅当同时满足两取等条件时，等号成立。

翻译一下这两个取等条件。$\sum_{x\in\mathcal{M}}Q(x) = 1$ 说明 $Q(x)=0,x \notin \mathcal{M}$ . 注意到分布 $P$ 也满足 $P(x)=0,x\notin \mathcal{M}$ . 再考虑第二个取等条件 $Q(x)=P(x), x\in\mathcal{M}$ ，从而得到统一的取等条件：

$$Q(x) = P(x), x\in\mathcal{X}$$

### 非对称性

KL 散度不是一个距离函数，它是非对称的，即

$$D_{KL}(P||Q) \neq D_{KL}(Q||P)$$

## 理解

KL 散度可以表示为

$$\begin{aligned}
D_{KL}(P||Q)&=\mathbb{E}_{x\sim P}\left[\ln\frac{P(x)}{Q(x)}\right]\\
&= \sum_{x\in\mathcal{X}}P(x)[-\ln Q(x)]-\sum_{x\in\mathcal{X}}P(x)[-\ln P(x)]
\end{aligned}$$

- $\sum_{x\in\mathcal{X}}P(x)[-\ln P(x)]$ 代表熵，即概率分布 $P$ 的平均信息量，或者说编码一个服从 $P$ 分布的样本所需的平均最优编码长度，单位为 nat；
- $\sum_{x\in\mathcal{X}}P(x)[-\ln Q(x)]$ 代表交叉熵，即把一个服从 $P$ 分布的样本错误地按照 $Q$ 分布进行编码时，所需的平均最优编码长度，单位为 nat .

因此，KL 散度就可以理解为，把一个服从 $P$ 分布的样本错误地按照 $Q$ 分布进行编码时，所“浪费”（冗余）的编码长度。“浪费”是因为我们没有按照 $P$ 分布，而是按照 $Q$ 分布进行编码。

综上所述，KL 散度实际是用“熵”这一概念，来描述两个分布的“距离”。
