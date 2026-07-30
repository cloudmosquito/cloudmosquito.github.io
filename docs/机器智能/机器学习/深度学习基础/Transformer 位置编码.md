# Transformer 位置编码

## Attention is all you need 里的 Sinusoidal 位置编码

对于输入矩阵的第 $pos$ 个向量，其第 $2i$ 和第 $2i+1$ 个分量分别为

$$\begin{aligned}
PE_{(pos,2i)} &= \sin\left(pos/10000^{2i/d}\right)\\
PE_{(pos,2i+1)} &= \cos\left(pos/10000^{2i/d}\right)
\end{aligned}$$

其中，$d=d_{model}$ 为输入向量的维度。在 Transformer 论文中用的是 512 . 

由于不带 Attention Mask 的 Attention 模型，对于输入矩阵里的每个向量是对称的，所以需要在输入的向量中加上位置编码向量，让模型能够利用 token 的位置信息。

> 论文原文："Since our model contains no recurrence and no convolution, in order for the model to make use of the order of the sequence, we must inject some information about the relative or absolute position of the tokens in the sequence."

假设模型 $f(\dots,x_m,\dots,x_n,\dots)$，加上编码向量后得到 $\tilde{f}(\dots,x_m+p_m,\dots,x_n+p_n,\dots)$ ，我们希望 $p_m$ 和 $p_n$ 能够引入 **相对位置信息** ，即 m-n 这一信息。

我们将 $p_m$ 和 $p_n$ 视为扰动项展开，可以得到

$$\tilde{f} = f+p_m^{\top}\frac{\partial f}{\partial x_m}+p_n^{\top}\frac{\partial f}{\partial x_n}+\frac{1}{2}p_m^{\top}\frac{\partial^2 f}{\partial x_m^2}p_m+\frac{1}{2}p_n^{\top}\frac{\partial^2 f}{\partial x_n^2}p_n+p_m^{\top}\frac{\partial^2 f}{\partial x_m\partial x_n}p_n + \dots$$

第 1 项没有位置信息，第 2 到 5 项只有绝对位置信息，第 6 项同时包含 $p_m$ 和 $p_n$ ，我们将其记为 $p_m^\top H p_n$ ，希望其能表达一定的相对位置信息。

### 简化情形

先假设 $H = I$ 为单位矩阵，此时 $p_m^\top H p_n$ 可以视为两个位置编码向量的内积 $p_m^\top p_n=\left<p_m,p_n\right>$ .

#### $d = 2$ 的情形

假设输入向量都是 2 维的，因此 $p_m$ 和 $p_n$ 也都是 2 维的，可视为复数。

$$p_m^\top H p_n = p_m^\top p_n = \left<p_m,p_n\right> = \mathrm{Re}[p_mp_n^{*}]$$

我们希望存在某个函数使得

$$\left<p_m,p_n\right> = g(m-n)$$

假设我们能找到复数 $q_{m-n} = p_m p_n^{*}$ ，那么两边取实部就是上式了。设 $p_m = r_me^{i\phi_m}, p_n^{*} = r_ne^{-i\phi_n},q_{m-n}=R_{m-n}e^{i\Phi_{m-n}}$ ，可以解得：

$$r_mr_ne^{-i\phi_n}e^{i\phi_m}=R_{m-n}e^{i\Phi_{m-n}} \Rightarrow \begin{cases}R_{m-n}=r_mr_n\\ \Phi_{m-n}=\phi_m-\phi_n \end{cases}$$

> 这里停一下，思考一下我们在干什么。
> 
> 我们需要求位置编码的形式，也就是 $p_m,p_n$ 的形式，也就是 $r_m,\phi_m,r_n,\phi_n$ 的形式。
> 
> 我们的约束条件是， $\left<p_m,p_n\right>$ 是一个关于 $m-n$ 的函数。方便起见，我们将这个函数也写成复数形式 $q_{m-n}$ ，从而推导出来了两个重要约束：$R_{m-n}=r_mr_n$ 和 $\Phi_{m-n}=\phi_m-\phi_n$ . 
> 
> 接下来，我们要利用这两个约束，求出 $r_m,\phi_m,r_n,\phi_n$ 的一组可行解。

对于第一个约束，令 $m=n$ ，得 $r_m^2=r_n^2=R_0$ 为一个常数，简单起见不妨设为 1 .

对于第二个约束，令 $m = n+1$ ，得 $\phi_{n+1}-\phi_n=\Phi_1$ 是一个等差数列，简单起见不妨设 $\phi_n = n\theta$ . 于是有 

$$p_n = e^{in\theta} = \begin{pmatrix}\cos n\theta\\ \sin n\theta\end{pmatrix}$$

---

换一个简单一点的视角。

考虑 $p_m = \begin{pmatrix}x_m\\y_m\end{pmatrix}, p_n=\begin{pmatrix}x_n\\y_n\end{pmatrix}$ ，我们希望 $p_m^\top p_n=x_mx_n+y_my_n = g(m-n)$ .

这个式子让我们想到 $\cos(m-n) = \cos m\cos n+ \sin m \sin n$ . 只把下标放在三角函数里面似乎不太好，我们乘一个 scale 吧，于是就有了 $\cos[(m-n)\theta] = \cos m\theta\cos n\theta+ \sin m\theta \sin n\theta$ .

于是就有 $p_m = \begin{pmatrix}\cos m\theta\\ \sin m\theta\end{pmatrix},p_n = \begin{pmatrix}\cos n\theta\\ \sin n\theta\end{pmatrix}$ .

#### $d$ 为任意偶数维的情形

> $d$ 是我们设计的，一般都是 2 的幂，Transformer 论文里用的是 512 .

由于向量内积 $\left<p_m,p_n\right>$ 时，每个分量都是独立的，所以我们可以两两一组按 $d = 2$ 的情形分析，通过调整 $\theta$ 的数值将其错开，于是有：

$$p_n = \begin{pmatrix}e^{in\theta_0}\\ e^{in\theta_1}\\ \vdots\\ e^{in\theta_{d/2-1}}\end{pmatrix} = \begin{pmatrix}\cos n\theta_0\\ \sin n\theta_0\\ \cos n\theta_1\\ \sin n\theta_1\\ \vdots\\ \cos n\theta_{d/2-1}\\ \sin n\theta_{d/2-1}\end{pmatrix}$$

---

至此，我们求得了一种位置编码形式，该形式可在 $H = I$ 为单位矩阵时引入相对位置信息。

关于 $H$ 不为单位阵的一般情况，苏剑林在博客中有简单的讨论，这里不展开了。

### 远程衰减

Transformer 论文里取 $\theta_i = 10000^{-2i/d}$ ，这个选择有什么意义？考虑内积

$$\begin{aligned}
\left<p_m,p_n\right> &= \mathrm{Re}\left[e^{\mathrm{i}(m-n)\theta_0}+e^{\mathrm{i}(m-n)\theta_1}+\cdots+e^{\mathrm{i}(m-n)\theta_{d/2-1}}\right]\\
&= \mathrm{Re}\left[\sum_{i=0}^{d/2-1}e^{\mathrm{i}(m-n)10000^{-i/(d/2)}}\right]\\
&= \frac{d}{2}\mathrm{Re}\left[\sum_{i=0}^{d/2-1}e^{\mathrm{i}(m-n)10000^{-i/(d/2)}}\frac{1}{d/2}\right]\\
\end{aligned}$$

> 注意到，中括号内是一个特殊形式的黎曼和，黎曼和的极限是定积分。
> 
> $$\int_{a}^{b}f(x)\mathrm{d}x = \lim_{m\to\infty}\sum_{i=0}^{m-1}f\left(a+\frac{b-a}{m}i\right)\cdot\frac{b-a}{m}$$

因此，$\left<p_m,p_n\right> \sim \dfrac{d}{2}\mathrm{Re}\left[\int_{0}^{1}e^{i(m-n)10000^{-t}}\mathrm{d}t\right]$ .

分析之后有以下结论：

1. $|m-n|$ 越大，内积 $\left<p_m,p_n\right>$ 越小。这符合我们的直觉，相对距离越大的两个 token，其相关性应该越弱。
2. 几乎每个定义域和值域在 $[0,1]$ 上的单调光滑函数都能使得上述积分有渐进衰减趋势， $10000^{-t}$ 不一定是最好的。

## 旋转式位置编码 Rotary Position Embedding

> “Sinusoidal位置编码是一种“想要成为相对位置编码的绝对位置编码”。一般来说，绝对位置编码具有实现简单、计算速度快等优点，而相对位置编码则直接地体现了相对位置信号，跟我们的直观理解吻合，实际性能往往也更好。……Sinusoidal位置编码隐约做到了这一点，但并不够好。”

在 Attention 计算过程中，核心运算是 q 和 k 的内积。我们希望给 q 和 k 添加绝对位置信息：

$$\tilde{q}_m = f(q,m),\ \tilde{k}_n=f(k,n)$$

并且希望内积的结果带有相对位置信息，也就是说希望存在恒等关系：

$$\left<f(q,m),f(k,n)\right>=g(q,k,m-n)$$

我们要求出一个满足恒等关系的 $f$ 形式。简单起见，我们设 $f(q,0)=q$ 和 $f(k,0)=k$ .

### $d=2$ 的情形

同样地，考虑二维情形，借助复数求解，设

$$\begin{aligned}
f(q,m)&=R_f(q,m)e^{i\Theta_f(q,m)} \\
f(k,n)&=R_f(k,n)e^{i\Theta_f(k,n)} \\
g(q,k,m-n)&=R_g(q,k,m-n)e^{i\Theta_g(q,k,m-n)}\\
\end{aligned}$$

有 $f(q,0)= \|q\|e^{i\Theta(q)}$ 和 $f(k,0)= \|k\|e^{i\Theta(k)}$ . 根据 $\mathrm{Re}[f(q,m)f^{*}(k,n)]=g(q,k,m-n)$ 得到两个约束条件：

$$\begin{aligned}
R_f(q,m)R_f(k,n) &= R_g(q,k,m-n)\\
\Theta_f(q,m)-\Theta_f(k,n) &= \Theta_g(q,k,m-n)
\end{aligned}$$

> 现在有复数的模和幅角两处地方能表达相对位置信息，而我们需要表达的相对位置信息只有 m-n 一个，所以约束过多了，我们可以简化掉一个。把第一个简化掉是很自然的。

对于第一个约束条件，代入 $m=n$ ，得到

$$R_f(q,m)R_f(k,m) = R_g(q,k,0) = R_f(q,0)R_f(k,0)=\|q\|\|k\|$$

因此，我们可以很自然地设 $R_f(q,m)=\|q\|$ 和 $R_f(k,m) = \|k\|$ .

对于第二个约束条件，同样代入 $m=n$ ，得到

$$\begin{aligned}
&\Theta_f(q,m)-\Theta_f(k,m) = \Theta_g(q,k,0) = \Theta_f(q,0)-\Theta_f(k,0) = \Theta(q)-\Theta(k)\\
\Rightarrow &\Theta_f(q,m)-\Theta(q)= \Theta_f(k,m)-\Theta(k)
\end{aligned}$$

简单起见可以认为，$\Theta_f(x,m)-\Theta(x)$ 应该是一个只和 m 相关而与 x 无关的函数（x = q 或 k 的值相同），记为 $\phi(m)$ . 则有 $\Theta_f(x,m) = \Theta(x) + \phi(m)$ . 易知 $\phi(0) = 0$ .

接下来，对第二个约束条件，代入 $n=m-1$ ，有

$$\begin{aligned}
&\Theta(q)+\phi(m)-\Theta(k)-\phi(m-1)=\Theta_{g}(q,k,1)\\
\Rightarrow &\phi(m)-\phi(m-1) = \Theta_{g}(q,k,1)+\Theta(k)-\Theta(q)
\end{aligned}$$

这说明 $\{\phi(m)\}$ 是等差数列，设右端为 $\theta$ ，就有 $\phi(m) = m\theta$ .

综上，我们推导出来的含位置编码的 $\tilde{q}_m$ 和 $\tilde{k}_n$ 分别为

$$\begin{aligned}
\tilde{q}_m = f(q,m)=\|q\|e^{i[\Theta(q)+m\theta]} = qe^{im\theta}\\
\tilde{k}_n = f(k,n)=\|k\|e^{i[\Theta(k)+n\theta]} = ke^{in\theta}
\end{aligned}$$

这个形式就是旋转了原向量，所以取名为“旋转式位置编码”。写成矩阵形式，有

$$\tilde{q}_m = \begin{bmatrix}\cos m\theta & -\sin m\theta\\ \sin m\theta & \cos m\theta\end{bmatrix}\begin{bmatrix}q_1\\q_2\end{bmatrix}
$$

### $d$ 为任意偶数的情形：

$$\tilde{q}_m = \underbrace{\begin{bmatrix}\cos m\theta_0 & -\sin m\theta_0 & 0 & 0 & \cdots & 0 & 0\\\sin m\theta_0 & \cos m\theta_0 & 0
 &0&\cdots&0&0 \\ 0& 0& \cos m\theta_1 & -\sin m\theta_1 & \cdots & 0 & 0\\ 0& 0& \sin m\theta_1 & \cos m\theta_1 & \cdots & 0 & 0\\ \vdots & \vdots& \vdots& \vdots& \ddots & \vdots& \vdots\\ 0 & 0 & 0 & 0 & \cdots & \cos m\theta_{d/2-1} & -\sin m\theta_{d/2-1}\\ 0 & 0 & 0 & 0 & \cdots & \sin m\theta_{d/2-1} & \cos m\theta_{d/2-1}\end{bmatrix}}_{\mathcal{R}_m}\begin{bmatrix}q_0\\q_1\\q_2\\\vdots\\q_{d-1}\end{bmatrix}$$

也就是说，只要给位置为 m 的向量 q 乘上 $\mathcal{R}_m$ ，位置为 n 的向量 k 乘上 $\mathcal{R}_n$ ，用变换后的 Q, K 矩阵做 Attention，结果就自动包含了相对位置信息，因为

$$(\mathcal{R}_mq_m)^{\top}(\mathcal{R}_nk_n) = q_m^{\top}\mathcal{R}_m^{\top}\mathcal{R}_nk_n = q_m^{\top}\mathcal{R}_{n-m}k_n$$

> 最后一个等式从复数或者旋转矩阵的角度去看比较好理解。

注意到 $\mathcal{R}_{m}$ 是一个分块正交对角矩阵，所以不会改变向量的模长，通常不会影响模型的稳定性。

实际运算过程中，考虑到稀疏矩阵乘法很浪费算力，推荐的实现形式为：

$$\tilde{q}_m=\begin{bmatrix}\cos m\theta_0\\ \cos m\theta_0\\\cos m\theta_1\\ \cos m\theta_1\\\vdots\\\cos m\theta_{d/2-1}\\ \cos m\theta_{d/2-1}\end{bmatrix}\otimes\begin{bmatrix}q_0\\q_1\\q_2\\q_3\\\vdots\\q_{d-2}\\q_{d-1}\end{bmatrix}+\begin{bmatrix}\sin m\theta_0\\ \sin m\theta_0\\ \sin m\theta_1\\ \sin m\theta_1\\ \vdots\\ \sin m\theta_{d/2-1}\\ \sin m\theta_{d/2-1}\end{bmatrix}\otimes\begin{bmatrix}-q_1\\q_0\\-q_3\\q_2\\\vdots\\-q_{d-1}\\q_{d-2}\end{bmatrix}$$

上式中的 $\otimes$ 代表逐位相乘。

### 远程衰减

在 $\theta_i$ 的选择上，RoPE 沿用 Sinusoidal 位置编码的方案，采用了 $\theta_i = 10000^{-i/2d}$ ，因为它具有一定的远程衰减性，具体分析看苏神博客。

## 参考资料

[1] 苏剑林. (Mar. 08, 2021). 《Transformer升级之路：1、Sinusoidal位置编码追根溯源 》[Blog post]. Retrieved from [https://spaces.ac.cn/archives/8231](https://spaces.ac.cn/archives/8231)
[2] 苏剑林. (Mar. 23, 2021). 《Transformer升级之路：2、博采众长的旋转式位置编码 》[Blog post]. Retrieved from [https://spaces.ac.cn/archives/8265](https://spaces.ac.cn/archives/8265)