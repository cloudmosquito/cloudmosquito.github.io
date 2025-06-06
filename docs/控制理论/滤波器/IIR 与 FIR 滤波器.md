# IIR 与 FIR 滤波器

IIR（Infinite Impulse Response，无限冲激响应）滤波器和 FIR（Finite Impulse Response，有限冲激响应）滤波器是相当常见的数字滤波器。

我们先从 Z 域给出两个滤波器的形式。FIR 滤波器的形式是

$$\begin{aligned}
\text{FIR 滤波器：} H(z) &= \sum_{i = 0}^{N}a_iz^{-i}\\
\text{IIR 滤波器：} H(z) &= \frac{\sum_{j=0}^Mb_jz^{-j}}{1+\sum_{i=1}^{N}a_iz^{-i}}
\end{aligned}\tag{1}$$

转化成离散表达式为

$$\begin{aligned}
\text{FIR 滤波器：} y\left[n\right] &= \sum_{i = 0}^{N}a_ix\left[n-i\right]\\
\text{IIR 滤波器：} y\left[n\right] &= \sum_{j=0}^Mb_jx\left[n-j\right] - \sum_{i=1}^{N}a_iy\left[n-i\right]
\end{aligned} \tag{2}$$

## 名称的来源

从（2）式就能看出，在 FIR 滤波器中，一个时刻的输入信号只能影响其后 $N$ 个时刻的输出信号，其**响应是有限的**；在 IIR 滤波器中，输出信号和之前时刻的输出信号存在递推关系，因此任意时刻的输入信号，都能永久影响未来的所有输出信号，其**响应是无限的**。

## 怎么设计 FIR / IIR 滤波器

最常用的方法是先设计一个能满足要求的模拟滤波器，比如巴特沃斯滤波器、切比雪夫滤波器等。然后利用双线性变换将其转换到 Z 域。做双线性变换时，需要注意频率的畸变问题。相关细节参见【[陷波器说明](./陷波器说明.md)】。

还有一种简单的方法，用 MATLAB 提供的滤波器设计工具，指定滤波器性能，让其输出一个可行的滤波器。

## 关于稳定性

结论是：**若 $H(z)$ 的极点在单位圆内，则滤波器在理论上是稳定的。**

按照该定理，FIR 滤波器必然是稳定的，IIR 滤波器可能会发散。这很符合直觉：FIR 输出就是前几个时刻输入的线性组合，肯定不会发散；但是 IIR 输出是滤波器运行以来所有输入的线性组合，的确可能发散。

怎么理解 “极点在单位圆内就稳定” 呢？我们回到（1）式，对 IIR 滤波器的形式做部分分式展开：

$$\begin{aligned}
H(z) &= \frac{\sum_{j=0}^{M}b_jz^{-j}}{1+\sum_{i=1}^{N}a_iz^{-i}}\\
&= \frac{B_1}{1-A_1z^{-1}} + \frac{B_2}{1-A_2z^{-1}} + \\
&\ \ \ \ \ \frac{B_3}{1-A_3z^{-1}} + \frac{B_3'}{(1-A_3z^{-1})^2} + \cdots + \frac{B_k}{1-A_kz^{-k}}
\end{aligned}$$

根据 Z 逆变换的线性性质，滤波器输出可以看作若干个小输出的线性组合。

$$\begin{aligned}
\frac{Y_1(z)}{X_1(z)} &= \frac{B_1}{1-A_1z^{-1}}\\
y_1\left[n\right]-A_1y_1\left[n-1\right] &= B_1x_1\left[n\right]\\
y_1\left[n\right] &= B_1x_1\left[n\right]+A_1y_1\left[n-1\right]
\end{aligned}$$

这里的 $A_1$ 就是极点。如果极点不在单位圆内，即 $\left|A_1\right| > 1$ ，则对任意的 $x_1\left[n\right]$ ， $y_1\left[n\right]$ 发散的概率几乎是 100% 。

> 以上讨论忽略了极点有虚部的情况，因为我们只是想对该定理建立一个直观认识。人生苦短，严谨证明就免了吧。

## 相位滞后

什么是相位滞后？直观地说，就是滤波后的信号相较于输入信号，整体有滞后。

首先我们必须认识到，实时滤波必然存在相位滞后。滤波器必须先收到待滤波信号，才能输出滤波后信号，其中的信号传输与计算时间不可避免。零相位滞后，代表着滤波器有准确预见未来的能力——就当下人类对时空的认识而言，这是不可能的。

还有一种情况，是已知一整段时间内的信号，要求滤波器输出这一整段时间信号的滤波结果。此时，我们已知未来信号，可以实现零相位滞后滤波。

### 零相位滞后滤波

一种很常见的做法是双向滤波。

**STEP 1：** 先对输入信号 $x\left[n\right]$ 滤一遍波，得到一组中间输出信号 $y_{temp}\left[n\right]$，该信号相对于输入信号存在相位滞后。

**STEP 2：** 把中间输出信号整体时间倒转 $y_{temp}\left[-n\right]$ ，再滤一遍波，得到一个倒转的输出信号 $y[-n]$ 。该倒转输出信号相对于 $y_{temp}[-n]$ 存在相位滞后，相对于 $y_{temp}[n]$ 就存在相位超前，相对于 $x[n]$ 就零相位滞后。

**STEP 3：** 把倒转输出信号倒转回来，得到输出信号 $y\left[n\right]$ 。

![双向滤波](./IIR%20与%20FIR%20滤波器.assets/双向滤波.png)

#### 滤波效果的变化

注意到，离散时域的时间倒转，相当于 Z 域对 $z$ 取倒数。对上述双向滤波过程，在 Z 域做如下分析：

$$\begin{aligned}
Y_{temp}(z) &= H(z)X(z)\\
Y(z^{-1}) &= H(z)Y_{temp}(z^{-1})\\
&= H(z)H(z^{-1})X(z^{-1})\\
Y(z) &= H(z^{-1})H(z) X(z)
\end{aligned}$$

这相当于把滤波器的传递函数从 $H(z)$ 变成了 $H(z^{-1})H(z)$ 。我们考虑这个“新滤波器”的幅频响应

$$\left|H(e^{j\omega T_s})H(e^{-j\omega T_s})\right| = \left|H(e^{j\omega T_s})\right|^2 \tag{3}$$

这里需要一点洞察力，但也不难。对虚部取负号不改变整个复数的模。

根据（3）式结论，双向滤波在频域上等效于做两次单向滤波。

#### 延拓

考察（2）式，要得到 $k$ 时刻的滤波信号，必须先知道 $k-1,k-2,\cdots$ 时刻的输入信号（IIR 滤波器还需要知道输出信号）。给定一整段时间的信号，对于开头部分，我们不知道前若干个时刻的信号。如果我们不处理这一问题，会导致滤波器输出具有严重的失真或者突变。

因此，我们需要**先对该段信号做延拓，滤完波后，再去除延拓部分**。对于双向滤波，要做双向延拓。

最省事的延拓方式就是将先前数据全部视为 0 （零延拓），或者视为起始时刻值（常值延拓），但是显然效果不会太好。一种比较优雅的方式是镜像延拓，即延拓信号与真实信号关于起始值对称。镜像延拓能保证信号的**一阶导数连续**。

$$\begin{aligned}
&\left[1,2,4,7,3,\cdots\right] \overset{\text{零延拓}}\Rightarrow &\left[0,0,0,0|1,2,4,7,3\cdots\right]\\
&\left[1,2,4,7,3,\cdots\right] \overset{\text{常值延拓}}\Rightarrow &\left[1,1,1,1|1,2,4,7,3\cdots\right]\\
&\left[1,2,4,7,3,\cdots\right] \overset{\text{镜像延拓}}\Rightarrow &\left[-1,-5,-2,0|1,2,4,7,3\cdots\right]
\end{aligned}$$
