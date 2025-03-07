# 云台 PID 控制理论分析

## Intro

本文以 Robomaster 中常见的英雄机器人云台 Pitch 轴控制为例，重点分析常规电机角度控制时的 PID 设计。

在分析时，本文忽略了执行器能力有限，PID输出应当限幅的问题；不考虑微分项、积分项在工程实践中的近似处理；不考虑阻力和外界扰动（假设前馈项能完美抵消）；不考虑工程实践时反馈数据的噪声部分。这些问题或许会留到之后探讨。

本文需要一点常微分方程、 Laplace 变换和复变函数的知识，如果读者没有接触过，可以先阅读一下附录，或许有所帮助。

## 1 建模

我们取云台水平位置为云台角度 $\theta$ 的零位，向上转为正，向下转为负。假定云台重心位于 $\theta_0$ 的位置，重力为 $G$，与转动轴心连线长度为 $l$。假定阻力矩恒定为 $f$，正负号与转动方向相反。用 $J$ 表示云台绕 Pitch 轴转动的转动惯量，用 $T$ 表示云台 Pitch 轴电机的输入力矩。从而得到

$$\ddot{\theta} = \frac{1}{J}(T - Gl\cos(\theta_0 + \theta)-f)$$

我们采用 PID 控制器 + 前馈的策略计算输入力矩 $T$，从而控制云台角度 $\theta$。【**假设我们的前馈是完美前馈，能完全抵消掉重力矩和阻力矩**】（注：这是本文的一个重要假设，所有分析都建立在该假设成立的情况下）。那么，我们有：

$$\ddot{\theta} = \frac{1}{J}T$$

分析我们常见的云台 Pitch 控制方案，控制板读取遥控器/键鼠指令后，在原有 Pitch 期望角度信号上加上一个值，因此 Pitch 期望角度信号可以看成是一个阶跃变化的信号。我们只分析一次阶跃变化时的请况，设这个阶跃变化量 $\Delta\theta_{ref} = r$.

施加新的期望信号的同时，云台的 Pitch 往往同时具有一个不为 0 的期望角度/实际角度/实际角速度/实际角加速度初值。我们记这些初值为 $\theta_{ref}(0) = A, \theta(0) = a, \dot\theta(0) = a_{d}, \ddot\theta(0) = a_{dd}$ .

## 2 PID 控制器

PID 控制器在复频域上可以表示为

$$\text{PID}(s) = K_c\left(1+\frac{1}{T_is} + T_ds\right) = K_c\frac{T_ds^2+s+1/T_i}{s}$$

其中，若积分时间 $T_i \to \infty$，则相当于没有积分项；若微分时间 $T_d = 0$，则相当于没有微分项。我们默认 PID 里的参数都大于 0.

## 3 单环 PID 稳定性分析

如果我们用一个单环的 PID 控制器来控制 Pitch，控制策略可以表示为

$$(\theta_{ref} - \theta)\cdot \text{PID} = J\ddot{\theta}$$

对等式两边做拉普拉斯变换，代入相关符号和 PID 控制器表达式，有

$$(\Theta_{ref}-\Theta)\cdot \text{PID} = J(s^2\Theta-s\theta(0)-\dot{\theta}_0)$$

把相关符号代入，得到

$$(\Theta_{ref}-\Theta)\cdot \text{PID} = J(s^2\Theta-sa-a_{d})$$

可以解出 $\Theta$ 的表达式：

$$\begin{aligned}
(Js^2+\text{PID})\Theta &= \Theta_{ref}\cdot \text{PID}+J(sa+a_d)\\
\Theta &= \frac{\Theta_{ref}\cdot \text{PID}+J(sa+a_d)}{Js^2+\text{PID}}
\end{aligned}$$

代入 PID 控制器的表达式，得到

$$\begin{aligned}
\Theta &= \frac{\Theta_{ref}\cdot K_c\frac{T_dT_is^2+T_is+1}{T_is}+J(sa+a_d)}{Js^2+K_c\frac{T_dT_is^2+T_is+1}{T_is}}\\
&= \frac{\Theta_{ref}K_c(T_ds^2+s+1/T_i) + Jas^2+Ja_ds}{Js^3+K_cT_ds^2 + K_cs+K_c/T_i}
\end{aligned}$$

这里的 $\Theta_{ref} = \mathcal{L}(\theta_{ref}) = \mathcal{L}(\theta_{ref}(0) + \Delta\theta_{ref}) = \mathcal{L}(A+r) = (A+r)/s$，$\mathcal{L}(\cdot)$ 表示 Laplace 变换 . 因此有

$$\Theta = \frac{Jas^3 + [(A+r)K_cT_d+Ja_d]s^2 + (A+r)K_cs+(A+r)K_c/T_i}{s(Js^3+K_cT_ds^2 + K_cs+K_c/T_i)} \tag{3.1}$$

$$\frac{\Theta}{\Theta_{ref}} = \frac{\frac{Ja}{A+r}s^3 + \left(K_cT_d+\frac{Ja_d}{A+r}\right)s^2 + K_cs+K_c/T_i}{Js^3+K_cT_ds^2 + K_cs+K_c/T_i} \tag{3.2}$$

### 3.1 暂态过程的稳定性

(3.2) 式对角相乘后，Laplace 反变换可以得到

$$JD^3\theta + K_cT_dD^2\theta + K_cD\theta + \frac{K_c}{T_i}\theta = \frac{Ja}{A+r}D^3\theta_{ref} + \left(K_cT_d+\frac{Ja_d}{A+r}\right)D^2\theta_{ref} + K_cD\theta_{ref} +\frac{K_c}{T_i}\theta_{ref} + C \tag{3.3}$$

其中，$D^n\theta = \frac{\mathrm{d}^2\theta(t)}{\mathrm{d}t^2}$，$C$ 是一个常数项。

根据常微分方程的知识，$\theta$ 的解可以由两部分相加得到：$\theta = \theta_{ss} +  \theta_{t}$ . 其中，$\theta_{ss}$ 是 (3.3) 微分方程的一个特解，又叫稳定解。$\theta_t$ 是 (3.3) 微分方程的一个通解，也叫暂态解，也就是 (3.4) 方程的解：

$$JD^3\theta + K_cT_dD^2\theta + K_cD\theta + \frac{K_c}{T_i}\theta = 0 \tag{3.4}$$

通常情况下，稳态解描述了 $t \to \infty$ 时 $\theta$ 的样子，而暂态解主要描述了从开始变化到最终稳定之间的过渡过程中 $\theta$ 的样子。我们先来看看暂态解。

根据常微分方程的知识，这个暂态解的形式是 $\theta(t) = \sum C_ie^{\lambda_it}$ ，$\lambda_i$ 是复数，如果这个和式解中存在一个 $\lambda_i = a+bj$ （虚部不为零）的部分，那么必然同时存在一个 $\lambda_j = a-bj$ 的共轭部分。

这里所有的 $\lambda$ ，如果【**实部全都小于 0**】 ，说明 $t\to \infty$ 的时候暂态解会趋近于 0，那么这个 $\theta(t)$ 就是【**收敛的、稳定的**】；如果【**存在一个实部大于 0 的**】$\lambda$，说明 $t \to \infty$ 的时候暂态解会趋近于 $\infty$，那么这个 $\theta(t)$ 就是【**发散的、不稳定的**】；如果【**存在两个实部为 0 ，虚部不为 0 的】** $\lambda_i$ 和 $\lambda_j$ ，那么暂态解里面就可以凑个三角函数出来，$\theta(t)$ 就会有一个【**等幅振荡**】的成分.

取 $\sum C_ie^{\lambda_it}$ 的其中一项代入 (3.4) 方程，可以得到：

$$Ce^{\lambda}\left(J\lambda^3+K_cT_d\lambda^2+K_c\lambda+\frac{K_c}{T_i}\right) = 0$$

显然，所有的 $\lambda$ 都是 $J\lambda^3+K_cT_d\lambda^2+K_c\lambda+K_c/T_i = 0$ 的根。这里需要注意一下，如果 $\lambda$ 存在一个 p 重根的话，暂态解的形式略有区别，这个 p 重根 $\lambda$ 会对应于 $C_1e^{\lambda t} + C_2te^{\lambda t} + ... + C_{p}t^{p-1}e^{\lambda t}$.

不难发现，$J\lambda^3+K_cT_d\lambda^2+K_c\lambda+K_c/T_i = 0$ 这个关于 $\lambda$ 的方程，和 【**$\Theta / \Theta_{ref}$ 的分母等于 0 这个关于 $s$ 的方程**】 ，本质上是同一个方程，根都是一样的，我们管这个方程叫作【**特征方程**】，把特征方程的解叫作【**特征根**】。

### 3.2 稳定时的值是多少

我们一般利用 Laplace 变换的终值定理（参见附录）分析 $\theta(t)$ 稳定时的值是多少。

注意，要使用 Laplace 变换的终值定理，我们需要保证 $s\Theta(s)$ 在包含虚轴的右半平面上是解析的，也就是说，$s\Theta(s)$ 的分母在右半平面上没有根，其实就是特征方程的解的实部全都小于 0.

> 这正好和 3.1.1 节的内容对上了。其实想想也知道，$\theta(t)$ 必须得是稳定的，才能算它稳定时的值是多少。

### 3.3 单环纯比例控制

此时 $T_d = 0, T_i \to \infty$ .

$$\begin{aligned}
\frac{\Theta}{\Theta_{ref}} &= \frac{\frac{Ja}{A+r}s^3 + \frac{Ja_d}{A+r}s^2 + K_cs}{Js^3 + K_cs}\\
&= \frac{\frac{Ja}{A+r}s^2 + \frac{Ja_d}{A+r}s + K_c}{Js^2 + K_c}
\end{aligned} $$

特征方程为

$$Js^2 + K_c = 0$$

特征根为 $s = \pm i\sqrt{K_c/J}$ ，所以此时【 **$\theta(t)$ 是等幅振荡的**】。

### 3.4 单环 PD 控制

此时 $T_d \neq 0, T_i \to \infty$ .

$$\begin{aligned}
\frac{\Theta}{\Theta_{ref}} &= \frac{\frac{Ja}{A+r}s^3 + \left(K_cT_d+\frac{Ja_d}{A+r}\right)s^2 + K_cs}{Js^3+K_cT_ds^2 + K_cs}\\
&= \frac{\frac{Ja}{A+r}s^2 + \left(K_cT_d+\frac{Ja_d}{A+r}\right)s + K_c}{Js^2+K_cT_ds + K_c}
\end{aligned} $$

特征方程为

$$\begin{aligned}
&Js^2+K_cT_ds+K_c = 0\\
&\left(\lambda+\frac{K_cT_d+\sqrt{K_c^2T_d^2-4JK_c}}{2J}\right)\left(\lambda+\frac{K_cT_d-\sqrt{K_c^2T_d^2-4JK_c}}{2J}\right)=0
\end{aligned}$$

由于特征根的实部都小于 0，所以此时【 **$\theta(t)$ 是稳定的**】。接下来利用 Laplace 变换的终值定理算一下稳定时候的值：

$$\begin{aligned}\lim_{t\to \infty}\theta(t) &= \lim_{s \to 0}s\Theta(s)\\
&= \lim_{s \to 0}\frac{Jas^3 + [(A+r)K_cT_d+Ja_d]s^2 + (A+r)K_cs}{Js^3+K_cT_ds^2 + K_cs}\\
&= A+r
\end{aligned}$$

$A+r$ 是期望角度的初始值加上期望角度的变化量，也就是最终的期望角度。也就是说，在前馈足够好的情况下，【**只用单环 PD 就能使 Pitch 角度无稳态误差地跟随阶跃信号**】。

### 3.5 单环 PI 控制

此时 $T_d = 0, T_i$有界 .

$$\begin{aligned}\frac{\Theta}{\Theta_{ref}} &= \frac{\frac{Ja}{A+r}s^3 + \frac{Ja_d}{A+r}s^2 + K_cs+K_c/T_i}{Js^3 + K_cs+K_c/T_i}\end{aligned}$$

特征方程为

$$s^3 + \frac{K_c}{J}s + \frac{K_c}{JT_i} = 0$$

解这样一个三次方程有点太复杂了。我们分析一下吧。

首先，根肯定不会在虚轴上。因为倘若 $s = j\omega$，那么 $K_c/(JT_i)$ 这一项肯定无法抵消为 0。

其次，我们考虑这个三次方程的实数解。令 $h(s) = s^3 + K_c/J \cdot s + K_c/(JT_i)$，求导得到 $\dot{h}(s) = 3s^2 + K_c/J > 0$，所以 $h(s)$ 单调递增。又因为 $h(0) = K_c/(JT_i) >0, h(-\infty)  = -\infty$，所以这个三次方程只有一个小于 0 的实数解。

接下来，我们分析剩下的两个复数解。不妨把刚才发现的实数解记为 $s_1 = \zeta < 0$ . 改写方程为

$$(s-\zeta)(s^2+\zeta s+(K_c/J+\zeta^2)) = 0$$

因此，两个复根可以表示为

$$s_{2,3} = \frac{-\zeta \pm \sqrt{\zeta^2-4(K_c/J+\zeta^2)}}{2}$$

可以发现，这两个复根的实部都是正数。也就是说，此时【**$\theta(t)$ 是发散的**】。

并且，【**$h(0) = K_c/(JT_i)$ 越接近 0**】, $\zeta$ 就越接近 0，发散部分的幅度就越小，【**$\theta(t)$ 就越接近于等幅振荡**】。

### 3.6 单环 PID 控制

定性理解，相当于 PI + PD，参数选择恰当就能稳定，参数选择不当就会发散。

在稳定的情况下，计算其稳定时的值：

$$\begin{aligned}\lim_{t\to \infty}\theta(t) &= \lim_{s \to 0}s\Theta(s)\\
&= \frac{Jas^3 + [(A+r)K_cT_d+Ja_d]s^2 + (A+r)K_cs+(A+r)K_c/T_i}{Js^3+K_cT_ds^2 + K_cs+K_c/T_i}\\
&= A+r
\end{aligned}$$

没有稳态误差。

想分析清楚 $\theta(t)$ 在什么条件下稳定，什么条件下发散太过复杂，超出我能力范围了。

放个 MATLAB 求解程序在这里。

```MATLAB
% 定义参数
J = 1;  % Pitch 轴转动惯量
A = 1;  % 初始期望角度
a = 1;  % 初始角度
r = 1;  % 期望角度变化量
ad = 1; % 初始角速度
Kc = 1; % PID 比例项
Td = 4; % PID 微分时间
Ti = 1; % PID 积分时间

% 拉普拉斯变换表达式
syms s t
F = ( J*a*s^3 + ((A+r)*Kc*Td+J*ad)*s^2 + (A+r)*Kc*s+ (A+r)*Kc/Ti ) / ...
  (s*(J*s^3 + Kc*Td*s^2 + Kc * s + Kc/Ti));

% 计算反变换
f_t = ilaplace(F, s, t);

% 绘制f(t)图像
fplot(f_t, [0, 40])  
xlabel('Time (s)')
ylabel('f(t)')
title('f(t) vs Time')
grid on
```

### 3.7 小结

综上所述，【**单环 PID 控制的稳定性相当差，很容易发散**】。这也是为什么我们要做双环 PID 控制。

## 4 双环 PID 分析

对于云台 Pitch 角度的双环 PID 控制（又叫串级控制）策略，一般是角度环+速度环，即一个 PID 根据角度误差计算出速度期望，另一个 PID 接收速度期望和实际速度，计算出输出力矩。（顺带一提，内环参数为什么选择“速度”？参见 附录 -2）

我们的双环 PID 控制策略可以表示为

$$[(\theta_{ref}-\theta)\cdot\text{PID}_{angle}-\dot{\theta}]\cdot\text{PID}_{vel} = J\ddot{\theta}$$

对等式两边做拉氏变换，可以得到

$$[(\Theta_{ref}-\Theta)\cdot \text{PID}_{angle}-s\Theta+\theta(0)]\cdot\text{PID}_{vel} = J(s^2\Theta-s\theta(0)-\dot{\theta}_0)$$

把相关符号代入，得到：

$$[(\Theta_{ref}-\Theta)\cdot \text{PID}_{angle}-s\Theta+a]\cdot\text{PID}_{vel} = J(s^2\Theta-sa-a_d) $$

可以解出 $\Theta$ 的表达式：

$$\Theta = \frac{\text{PID}_{vel}(\Theta_{ref}\cdot \text{PID}_{angle}+a)+Jsa+Ja_d}{Js^2+\text{PID}_{vel}\cdot \text{PID}_{angle}+\text{PID}_{vel}s} \tag{4.1}$$

### 4.1 速度环选什么？

速度环的控制策略可以表示为

$$\begin{aligned}
(\dot\theta_{ref}-\dot\theta)\cdot\text{PID}_{vel} &= J\ddot\theta\\
(\Theta_{dref}-\Theta_{d})\cdot \text{PID}_{vel} &= J(s\Theta_{d}-a_d)\\
(Js+\text{PID}_{vel})\cdot\Theta_{d} &= \Theta_{dref}\cdot \text{PID}_{vel} + Ja_d\\
\Theta_d &= \frac{\Theta_{dref}\cdot \text{PID}_{vel} + Ja_d}{Js+\text{PID}_{vel}}\\
&= \frac{\Theta_{dref}\cdot K_c(T_ds^2+s+1/T_i)/s + Ja_d}{Js+K_c(T_ds^2+s+1/T_i)/s}\\
&= \frac{\Theta_{dref}\cdot K_c(T_ds^2+s+1/T_i) + Ja_ds}{(J+K_cT_d)s^2+K_cs+K_c/T_i}\\
&= \frac{((V+a_d)K_cT_d+Ja_d)s^2+(V+a_d)K_cs+(V+a_d)K_c/T_i}{s((J+K_cT_d)s^2+K_cs+K_c/T_i)}
\end{aligned}$$

其中带 $d$ 下标的代表是角速度，$V$ 代表角速度期望值阶跃变化量。

用第 3 节的方法很容易分析出，【**不管怎么选择 PID 参数，速度环都不会发散，最终稳定时都没有稳态误差**】。

在 MATLAB 的帮助下，我们可以直接解出 $\dot\theta$ 的形式：

```MATLAB
% 拉普拉斯变换表达式
syms s t J ad Kc Td Ti V
F = ( ((V+ad)*Kc*Td + J*ad)*s^2 + (V+ad)*Kc*s + (V + ad)*Kc/Ti) / ...
  (s*((J+Kc*Td)*s^2 + Kc*s + Kc / Ti));
% 计算反变换
f_t = ilaplace(F, s, t);
```

稍微整理一下：

$$\begin{aligned}
\dot\theta &= V + a_d - \frac{J  V}{J  + K_c T_d} \exp\left(-\frac{K_c}{2(J+K_c T_d)}t\right) \left( \cosh\left( \omega t \right) - \frac{ \sinh\left( \omega t \right)}{\sqrt{1-4/T_i \cdot \left( T_d+J/K_c\right)}} \right)\\
\omega &= \frac{ \sqrt{1-4/T_i \cdot \left( T_d+J/K_c\right)} }{2 (J/K_c + T_d)}
\end{aligned}$$

先关注微分项参数 $T_d$ ，它主要和 $\frac{J  V}{J  + K_c T_d} \exp\left(-\frac{K_c}{2(J+K_c T_d)}t\right)$ 有关。 随着 $T_d$ 增大，暂态解的系数 $JV / (J+K_cT_d)$ 减小，指数中的系数 $-K_c/\left(2(J+K_c T_d)\right)$ 绝对值减小，这说明【**微分项的引入能够抑制过渡过程的振荡幅度，但是会延长过渡过程的持续时间**】。 这不好，因为我们希望内环参数变化灵敏，调节时间短。所以我们速度环不用微分项。

再关注积分项参数 $1/T_i$ ，它主要和 $\left( \cosh\left( \omega t \right) - \frac{ \sinh\left( \omega t \right)}{\sqrt{1-4/T_i \cdot \left( T_d+J/K_c\right)}} \right)$ 有关。 随着 $1/T_i$ 增大，$\omega$ 从 1 减小到 0 再变为一个虚数。先考虑 $\omega$ 是个正数的情况：

此时， $\left( \cosh\left( \omega t \right) - \frac{ \sinh\left( \omega t \right)}{\sqrt{1-4/T_i \cdot \left( T_d+J/K_c\right)}} \right)$ 可以写成 $(1-k)(e^{-kx} - e^{kx})/(2k)$ 的形式，这里的 $k = \sqrt{1-4/T_i \cdot \left( T_d+J/K_c\right)} < 1$，$x = t / [2(J/K_c + T_d)]$ ，是个随着 $k$ 减小而递增的值。

也就是说，**当 $T_i > 4(T_d+J/K_c)$ 时，$1/T_i$ 越大，过渡过程的振荡幅度越大**。

再考虑 $\omega$ 是个虚数，即 $\omega = \pm j W$ 的情况，由于 $\left( \cosh\left( \omega t \right) - \frac{ \sinh\left( \omega t \right)}{\sqrt{1-4/T_i \cdot \left( T_d+J/K_c\right)}} \right)$ 关于 $W$ 是个偶函数，所以我们只需要考虑 $W$ 为正数的情况，此时

$$\begin{aligned}
\left( \cosh\left( \omega t \right) - \frac{ \sinh\left( \omega t \right)}{\sqrt{1-4/T_i \cdot \left( T_d+J/K_c\right)}} \right) &= \cos(Wt) - \frac{j\sin(Wt)}{jW}\\
&= \cos(Wt)-\frac{\sin(Wt)}{W}\\
&= \frac{\sqrt{1+W^2}}{W}\cos(Wt+\phi), \phi = \arctan\frac{1}{W}
\end{aligned}$$

显然，这是一个等幅振荡的形式。随着 $1/T_i$ 增大，$W$ 也增大，$\sqrt{1+W^2}/W$ 趋近于 1，$\cos(Wt+\phi)$ 的角频率增大，相位角减小。

也就是说，**当 $T_i$ 从  $>4(T_d+J/K_c)$ 转变到 $<4(T_d+J/K_c)$ 后，过渡过程中出现了等幅振荡的部分，其稳定性完全依赖于 $\exp\left(-\frac{K_c}{2(J+K_c T_d)}t\right)$，调节时间延长了。随着 $1/T_i$ 继续增大，等幅振荡部分的幅度减小，振荡频率增大**。

综上所述，【**积分项的引入会导致过渡过程振荡加剧、延长**】。这也不好。所以我们速度环也不应该使用积分项。

留个速度环的 MATLAB 计算和画图程序在这里。

```MATLAB
% 定义参数
J = 1;  % Pitch 轴转动惯量
ad = 1; % 初始角速度
Kc = 1; % PID 比例项
Td = 0.1; % PID 微分时间
Ti = 0.01; % PID 积分时间
V = 5; % 期望角速度变化量

% 拉普拉斯变换表达式
syms s t
F = ( ((V+ad)*Kc*Td + J*ad)*s^2 + (V+ad)*Kc*s + (V + ad)*Kc/Ti) / ...
  (s*((J+Kc*Td)*s^2 + Kc*s + Kc / Ti));

% 计算反变换
f_t = ilaplace(F, s, t);

% 绘制f(t)图像
fplot(f_t, [0, 100])  
xlabel('Time (s)')
ylabel('f(t)')
title('f(t) vs Time')
grid on
```

### 4.2 双环 PID 性能分析

根据 4.1 节的结论，速度环的 PID，我们只应该使用比例环节，即 $\text{PID}_{vel} = K_p$ . 从而，我们改写 (4.1) 式，得到：

$$\Theta = \frac{K_p(\Theta_{ref}\cdot \text{PID}_{angle}+a)+Jsa+Ja_d}{Js^2+K_p\cdot \text{PID}_{angle}+K_ps}$$

代入 PID 控制器的表达式，得到：

$$\begin{aligned}\Theta &= \frac{K_p\Theta_{ref}K_c\frac{T_ds^2+s+1/T_i}{s}+K_pa+Jsa+Ja_d}{Js^2+K_pK_c\frac{T_ds^2+s+1/T_i}{s}+K_ps}\\
&= \frac{K_pK_c\Theta_{ref}(T_ds^2+s+1/T_i)+Jas^2+(K_pa+Ja_d)s}{Js^3 + K_pK_c(T_ds^2+s+1/T_i) + K_ps^2}\\
&= \frac{(K_pK_c\Theta_{ref}T_d+Ja)s^2+(K_pK_c\Theta_{ref}+K_pa+Ja_d)s+K_pK_c\Theta_{ref}/T_i}{Js^3+K_p(K_cT_d+1)s^2+K_pK_cs+K_pK_c/T_i}\end{aligned}$$

这里的 $\Theta_{ref} = \mathcal{L}(\theta_{ref}) = \mathcal{L}(A+r) = (A+r)/s$ . 因此有

$$\begin{aligned}\Theta &= \frac{\left( K_pK_cT_d\frac{A+r}{s}+Ja \right)s^2 + \left( K_pK_c\frac{A+r}{s}+K_pa+Ja_d \right)s + K_pK_c\frac{A+r}{sT_i}}{Js^3+K_p(K_cT_d+1)s^2+K_pK_cs+K_pK_c/T_i}\\
&= \frac{(K_pK_cT_d(A+r)+Jas)s^2 + K_pK_c(A+r)s+(K_pa+Ja_d)s^2 + K_pK_c(A+r)/T_i}{Js^4+K_p(K_cT_d+1)s^3+K_pK_cs^2+K_pK_cs/T_i}\\
&= \frac{Jas^3+(K_pK_cT_d(A+r)+K_pa+Ja_d)s^2 + K_pK_c(A+r)s+K_pK_c(A+r)/T_i}{s(Js^3+K_p(K_cT_d+1)s^2+K_pK_cs+K_pK_c/T_i)}\\
\end{aligned}$$

### 2.1 角度环纯比例

此时，$T_d = 0, T_i \to \infty$ .

$$\Theta = \frac{Jas^2+(K_pa+Ja_d)s+K_pK_c(A+r)}{s(Js^2+K_ps+K_pK_c)}$$

特征方程为

$$Js^2+K_ps+K_pK_c = 0$$

显然，两个特征根的实部都小于 0，所以此时【 **$\theta(t)$ 是稳定的**】。接下来利用 Laplace 变换的终值定理算一下稳定时候的值：

$$\lim_{t\to\infty}\theta(t) = \lim_{s\to0}s\Theta(s) = \frac{K_pK_c(A+r)}{K_pK_c} = A+r$$

可以发现，Pitch 实际角度最终与期望角度保持一致。也就是说，【**只要前馈足够理想，纯比例就能实现 Pitch 角度无稳态误差**】。

用 MATLAB 对 $\Theta(s)$ 做 Laplace 反变换，可以得到

$$\begin{aligned}
\theta(t) &= A + r - \left( \exp\left( -\frac{K_p}{2J}t \right) \left( (r-a+A)\cosh\left( \omega t\right) - \frac{\sinh\left( \omega t \right) \left( 2J a_d + K_p (a-r-A) \right)}{\sqrt{K_p^2 - 4J K_c K_p}} \right)\right)\\
\omega &= \frac{1}{2J}\sqrt{K_p^2-4JK_pK_c}
\end{aligned}$$

首先，观察暂态解指数项的系数 $K_p/(2J)$，可以发现此时暂态过程的持续时间与外环比例项 $K_c$ 无关，仅与内环比例项 $K_p$ 有关。【**内环比例项 $K_p$ 越大，暂态过程持续时间越短，暂态过程振荡幅度越小。下同**】。

考虑 $K_c$ 从 $<K_p/(4J)$ 的情况下逐渐增大，$\omega$ 将从一个正数逐渐减小至 0 ，再转变为一个虚数，虚部逐渐增大。

这里我不知道怎么分析，但是结果是，【**外环比例项 $K_c$ 越大，暂态过程振荡越剧烈**】。

### 2.2 角度环 PD

此时，$T_d \neq 0, T_i \to \infty$。

$$\Theta = \frac{Jas^2+(K_pK_cT_d(A+r)+K_pa+Ja_d)s+K_pK_c(A+r)}{s(Js^2+K_p(K_cT_d+1)s+K_pK_c)}$$

特征方程为

$$Js^2+K_p(K_cT_d+1)s+K_pK_c = 0$$

显然，两个特征根的实部都小于 0，$\theta(t)$ 稳定，并且可以用 Laplace 变换的终值定理算出稳定值为 $A+r$ .

可以发现，微分项的引入不改变控制器的稳态特性。我们分析一下暂态特性。

> ~~这个部分是最开始写的。刚开始不知道 MATLAB 能直接形式化做 Laplace 反变换，自己去分类讨论算了两天，结果还有错的哈哈~~

自己做 Laplace 反变换还是太吃操作了，还好我有简单有强势的 MATLAB ：

```MATLAB
% 拉普拉斯变换表达式
syms s t J a Kp Kc Td Ti A r ad
F = ( J*a*s^2 + (Kp*Kc*Td*(A+r) + Kp*a + J*ad)*s + Kp*Kc*(A+r) ) / ...
  ( s * (J*s^2 + Kp*(Kc*Td+1)*s + Kp*Kc ) );

% 计算反变换
f_t = ilaplace(F, s, t);
```

$$\begin{aligned}
\theta(t) &= A + r - \frac{1}{J^2\omega}\exp\left( -\frac{ \left( K_p + K_c K_p T_d \right) }{2 J} t \right) \left( \cosh\left( \omega t \right) - \sinh\left( \omega t \right) \left( \frac{J a_d}{r-a+A} + \frac{K_p(K_cT_d-1)}{2} \right) \right)\\
\omega &= \frac{K_p}{2J}\sqrt{K_c^2 T_d^2 + 2K_c T_d - 4J K_c/K_p + 1}
\end{aligned}$$

随着 $T_d$ 增大，$\omega$ 增大，暂态解系数 $1/(J^2\omega)$ 减小，暂态解指数项里的系数绝对值 $(K_p+K_cK_pT_d)/(2J)$
增大，说明【**微分项的引入能有效抑制暂态过程的振荡，但会延长暂态过程的持续时间**】。

### 2.3 角度环 PI

此时，$T_d = 0, T_i$有界。

$$\Theta= \frac{Jas^3+(K_pa+Ja_d)s^2 + K_pK_c(A+r)s+K_pK_c(A+r)/T_i}{s(Js^3+K_ps^2+K_pK_cs+K_pK_c/T_i)}$$

特征方程

$$Js^3+K_ps^2+K_pK_cs+K_pK_c/T_i = 0$$

令 $m(s) = Js^3+K_ps^2+K_pK_cs+K_pK_c/T_i$，求个导得到 $\dot{m}(s) = 3Js^2+2K_ps+K_pK_c$，分别考虑有两个实根的情况和没有两个实根的情况。

有两个实根时，这两个根分别为 $s = \frac{-K_p \pm K_p\sqrt{1-3K_c/K_p}}{3}$，都为负数，所以 $m(s)$ 在先增再减再增，又因为 $m(-\infty) = -\infty, m(0) > 0$，所以 $m(s)$ 的三个根都是负实数，$\Theta$ 是稳定的。

$\dot{m}(s)$ 没有两个实根时，$m(s)$ 单调递增，因为 $m(0) > 0$，所以 $m(s)$ 只有一个负实根 $\delta$，从而可以将 $m(s)$ 写成 $Js^2 + (K_p+J\delta)s+K_p(K_c+\delta)+J\delta^2$ ，显然另外两个复根的实部也是负数，因此 $\Theta$ 是稳定的。

稳态值是 $(A+r)$，无误差。

暂态过程分析不出来了。大概测试结果是【**积分项的引入会导致暂态过程振动幅度增大，暂态过程时间延长**】。

在不考虑扰动的情况下，积分的引入带来的似乎只有负面作用。

## 附录

### 0 前置知识
#### 0.1 描述线性系统的基本方式（微分方程）

对于一个线性系统 $G$，输入 $r(t)$，输出  $y(t)$，我们通常用微分方程的形式来描述这一系统：

$$D^ny + a_{n-1}D^{n-1}y + a_{n-2}D^{n-2}y + ... + a_{1}Dy + a_0y = b_mD^{m}r + b_{m-1}D^{m-1}r + ... +  b_1Dr + b_0r$$

其中，$a_{i}, b_{j}$ 是常系数，$D^{k}y$ 代表 $\frac{\mathrm{d}^ky(t)}{\mathrm{d}t^k}$。

如果是现实世界中存在的系统，考虑到**因果性**，必然有 $n > m$。

这样一个系统，输入信号后，输出信号可以分成两个部分，一个是**暂态输出**，指的是 $t\to \infty$ 后会变成 0 的部分；另一个是**稳态输出**，指的是 $t\to \infty$ 后仍然存在的部分。

#### 0.2 拉普拉斯变换

拉氏变换可以将连续的时域信号/函数转化为复频域的信号/函数。

对于一个时域上的函数 $f(t)$，定义 拉普拉斯变换（以下简称拉氏变换）：

$$F(s) = \mathcal{L}(f(t)) = \int_{0}^{+\infty}f(t)e^{-st}\mathrm{d}t$$

其中，$\mathcal{L}(f(t))$ 表示对 $f(t)$ 做拉氏变换。（注：拉氏变换要求 $f(t)$ 在 $(0,+\infty)$ 上有定义）

拉氏变换实际上是将时域上的函数转化到复频域，有一些特殊的性质，例如：

**时域位移相当于复频域乘以 $e^{-sT}$**。

$$\mathcal{L}(f(t-T)) = e^{-sT}F(s) + \int_{-T}^{0}f(p)e^{-sp}\mathrm{d}p$$

证明：

$$\begin{aligned}\int_{0}^{+\infty}f(t-T)e^{-st}\mathrm{d}t &\overset{p = t-T}{=} \int_{-T}^{+\infty}f(p)e^{-s(p+T)}\mathrm{d}p\\
&=e^{-sT}\int_{-T}^{+\infty}f(p)e^{-sp}\mathrm{d}p\\
&= e^{-sT}F(s) + \int_{-T}^{0}f(p)e^{-sp}\mathrm{d}p
\end{aligned}$$

只要 $f(t)$ 在 $t < 0$ 的部分均等于 0，那么上式就可以简化为

$$\mathcal{L}(f(t-T)) = e^{-sT} F(s)$$

**复频域上乘以 s 相当于时域求微分**。

$$\begin{aligned}
sF(s) - f(0) &= \mathcal{L}(\frac{\mathrm{d}f(t)}{\mathrm{d}t})\\
s^2F(s) - sf(0)-f'(0) &= \mathcal{L}(\frac{\mathrm{d}^2f(t)}{\mathrm{d}t^2})\\
\end{aligned}$$

证明：

$$\begin{aligned}
\mathcal{L}(\frac{\mathrm{d}f(t)}{\mathrm{d}t}) &= \int_0^{+\infty}\frac{\mathrm{d}f(t)}{\mathrm{d}t}e^{-st}\mathrm{d}t\\ &= \int_0^{+\infty}e^{-st}\mathrm{d}f(t)\\
&= e^{-st}f(t)|_{0}^{+\infty} - \int_0^{+\infty}f(t)\mathrm{d}e^{-st}\\
&= 0\cdot f(t)-1\cdot f(0) +s\int_0^{+\infty}f(t)e^{-st}\mathrm{d}t\\
&= -f(0) + sF(s)\\
\mathcal{L}(\frac{\mathrm{d}^2f(t)}{\mathrm{d}t^2}) &= \int_0^{+\infty}\frac{\mathrm{d}^2f(t)}{\mathrm{d}t^2}e^{-st}\mathrm{d}t\\ &= \int_0^{+\infty}e^{-st}\mathrm{d}(\frac{\mathrm{d}f(t)}{\mathrm{d}t})\\
&= e^{-st}\frac{\mathrm{d}f(t)}{\mathrm{d}t}|_{0}^{+\infty} - \int_0^{+\infty}\frac{\mathrm{d}f(t)}{\mathrm{d}t}\mathrm{d}e^{-st}\\
&= -f'(0) +s\int_0^{+\infty}\frac{\mathrm{d}f(t)}{\mathrm{d}t}e^{-st}\mathrm{d}t\\
&= -f'(0) + s(-f(0) + sF(s))\\
&= -f'(0) -sf(0) + s^2F(s)
\end{aligned}$$

其中，$F(s) = \mathcal{L}(f(t))$。在以上推导过程中，我们假定了 $f(0_+) = f(0_-)$。如果 $f(t)$ 的性质特别好，满足 $t = 0$ 时，$f(t)$ 的任意阶导数值均为 0，那么上式可以进一步简化为

$$\mathcal{L}(\frac{\mathrm{d}^n f(t)}{\mathrm{d}^t}) = s^{n}F(s)$$

拉氏变换可以用来分析微分方程，大大简化我们的分析难度。

比如，对于 1.1 节中的系统，我们输入 $r(t) = 1$，求输出 $y(t)$？直接解微分方程不太容易，但是用拉氏变换分析就会简单不少：

首先，我们在零初始条件下，将系统的微分方程等式两边做拉氏变换，得到

$$\begin{aligned}
s^nY(s) + a_{n-1}s^{n-1}Y(s) + ...+a_1sY(s)+a_0Y(s) &= b_ms^{m}R(s) + b_{m-1}s^{m-1}R(s)+...+b_1sR(s)+b_0R(s)\\
\frac{Y(s)}{R(s)} &= \frac{b_ms^m + b_{m-1}s^{m-1} + ... + b_0}{s^n + a_{n-1}s^{n-1} + ... + a_1s + a_0}
\end{aligned}$$

上面这个比值形式就是系统的**传递函数**。我们考虑一个简单一点的情况：

$$\frac{Y(s)}{R(s)} = \frac{1}{s + 1}$$

因此，我们有

$$Y(s) = R(s)\frac{1}{s+1} = \frac{1}{s}\frac{1}{s+1} = \frac{1}{s} - \frac{1}{s+1}$$

对上式做拉氏反变换，即可得到 $y(t) = 1 - e^{-t}$。（注：$\mathcal{L}(1) = \frac{1}{s}$ 很容易证明；拉氏反变换一般是直接查表）

#### 0.3 Laplace 变换初值定理和终值定理

若 $\mathcal{L}[f(t)] = F(s)$，$\mathrm{d}f/\mathrm{d}t$ 是可以 Laplace 变换的，且 $f(+\infty)$ 存在，$sF(s)$ 在包含虚轴的右半平面上解析（原点除外），则有终值定理：

$$f(+\infty) = \lim_{s \to 0}sF(s)$$

若 $\mathcal{L}[f(t)] = F(s)$，$\mathrm{d}f/\mathrm{d}t$ 是可以 Laplace 变换的，且 $\lim_{s\to \infty}sF(s)$ 存在，则有初值定理

$$f(0_+) = \lim_{s\to \infty}sF(s)$$

### -1 复变函数相关知识

复变函数中最重要的一个式子是欧拉公式 $e^{j\theta}=\cos\theta+j\sin\theta$，其中 $\cos\theta$ 是复数的实部，$j\sin\theta$ 是复数的虚部。根据欧拉公式，我们可以很简单地推导出，$j = e^{j\frac{\pi}{2}}$。

事实上，在复变函数的理论中，所有的复数 $\mathbf{A}$ 都可以由 $Ae^{j\theta}$ 的形式表示。$\mathbf{A}$ 可以理解为一个复平面上的矢量，从原点指向 $(A\cos\phi,A\sin\phi)$ 的位置。通常，我们用幅度 $A$ 和相位角 $\phi$ 来描述这个矢量。定义一个求幅度的运算：$|\mathbf{A}| = |Ae^{j\theta}| = A$。

任意一个复数乘以 $Ae^{j\theta}$，表示将该复数对应的复平面矢量的幅度乘以 $A$，角度转 $\theta$。相反地，除以 $Ae^{j\theta}$，就是幅度除以 $A$，角度反转 $\theta$。

根据上述知识，我们可以得到：

$$\left|\frac{Ae^{j\theta}}{Be^{j\phi}}\right| = \left|\frac{A}{B}e^{j(\theta-\phi)}\right| = \left|\frac{A}{B}\right| = \frac{|Ae^{j\theta}|}{|Be^{j\phi}|}$$

### -2 串级控制选取内环参数的原则

双环 PID 中，外环参数是固定的，我们想控制 Pitch 的角度，那么外环就必须接收角度的期望值和反馈量；但内环参数（比如”速度环“的”速度“）不是固定的，可以由设计者选择。一般来说，选取内环参数需要满足以下几点要求：

1. 可以被测量；
2. 比外环参数能更快地感知干扰；
3. 直接受执行器输出的影响；
4. 对外环参数有明确且较大的影响；
5. 比外环参数的反应更加灵敏，调节时间更短；
6. 最好包含系统的非线性或者时变部分。
