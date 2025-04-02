# 系统阶数与 PID

我们分别讨论以下三个系统的 PID 控制器设计原则：
  
$$\begin{aligned}
y_1 &= k_1u_1 - f\\
\dot{y}_2 &= k_2u_2 - f\\
\ddot{y}_3 &= k_3u_3 - f\\
\end{aligned}$$

其中， $y$ 是系统输出，可以被测量； $u$ 是控制输入， $k$是常系数， $f$ 是扰动项，简单建模为一个大于零的定值。

本文讨论是非常理想化的，建立在我们能完美无损求导、求积分的基础上。工程实际中，这是不可能的，因此控制结果与下文计算结果或有所出入，但定性性质一致。

## 零阶系统

### 比例控制 P

针对零阶系统 $y = ku-f$ ，倘若我们单纯采用比例控制，那么有

$$\begin{aligned}y &= k\cdot K_p(y_{ref} - y)-f\\
\overset{拉氏变换}\Rightarrow Y(s) &= kK_p(Y_{ref}(s) - Y(s)) - \frac{f}{s}\\
Y &= \frac{kK_p}{1+kK_p}Y_{ref}-\frac{f}{(1+kK_p)s}
\end{aligned}$$

对于阶跃期望值 $Y_{ref} = \dfrac{r}{s}$ ，系统响应为 $\dfrac{kK_pr-f}{(1+kK_p)s}$ ，转化到时域为 $\dfrac{kK_pr-f}{1+kK_p}$ ，是一个有稳态误差的阶跃值，并且必然没有超调。

## 比例积分控制 PI

为了消除这一稳态误差，我们需要采用 PI 控制，有

$$\begin{aligned}y &= k\left(K_p(y_{ref} - y) + K_iD^{-1}(y_{ref}-y)\right) - f\\
\overset{拉氏变换}\Rightarrow sY(s) &= kK_ps(Y_{ref}(s) - Y(s)) + kK_i(Y_{ref}(s)-Y(s)) - f\\
Y &= \frac{kK_ps+kK_i}{(1+kK_p)s+kK_i}Y_{ref}-\frac{f}{(1+kK_p)s+kK_i}\\
&= Y_{ref}-\frac{sY_{ref}+f}{(1+kK_p)s+kK_i}
\end{aligned}$$

对于阶跃期望值 $Y_{ref} = \dfrac{r}{s}$ ，系统响应为 $\dfrac{r}{s} - \dfrac{r+f}{(1+kK_p)s+kK_i}$ ，转化到时域就是 $r-\dfrac{r+f}{1+kK_p}e^{-\frac{kK_i}{1+kK_p}t}$ 。稳态误差是 0 ，必然没有超调。

> 需要注意，积分项会带来系统响应的“滞后”。“滞后”是根据“系统从接收到控制信号到输出稳定的时间”来判断的。

## 比例微分积分控制 PID

在此基础上，我们思考一下是否需要引入微分控制。倘若采用 PID 控制，有

$$\begin{aligned}y &= k\left( K_p(y_{ref} - y) + K_iD^{-1}(y_{ref}-y) + K_dD(y_{ref}-y) \right)-f\\
\overset{拉氏变换}\Rightarrow sY(s) &= kK_ps(Y_{ref}(s) - Y(s)) + kK_i(Y_{ref}(s)-Y(s)) + kK_ds^2(Y_{ref}(s)-Y(s))-f\\
Y &= \frac{(kK_ds^2+kK_ps + kK_i)Y_{ref}-f}{kK_ds^2+(1+kK_p)s+kK_i}\\
&= Y_{ref}-\frac{sY_{ref}+f}{kK_ds^2+(1+kK_p)s+kK_i}
\end{aligned}$$

对于阶跃期望值 $Y_{ref} = \dfrac{r}{s}$ ，系统响应为 $\dfrac{r}{s} - \dfrac{r+f}{kK_ds^2+(1+kK_p)s+kK_i}$ ，转化到时域就是 $r-\dfrac{r+f}{\sqrt{(1+kK_p)^2-4k^2K_iK_d}}e^{-\dfrac{1+kK_p}{2kK_d}t}\left(e^{\dfrac{\sqrt{(1+kK_p)^2-4k^2K_iK_d}}{2kK_d}t} - e^{-\dfrac{\sqrt{(1+kK_p)^2-4k^2K_iK_d}}{2kK_d}t}\right)$ ，稳态误差是 0 ，但是有超调。

要注意，上述形式的解只适用于 $(1+kK_p)^2-4k^2K_iK_d > 0$ 的过阻尼情况。如果是欠阻尼情况，过渡过程会有减幅振荡。

![Figure_1.png](./系统阶数与%20PID.assets/Figure_1.png)

## 一阶系统

### 比例控制 P

针对一阶系统 $\dot{y} = ku-f$ ，倘若我们单纯采用比例控制，那么有

$$\begin{aligned}\dot{y} &= k\cdot K_p(y_{ref} - y)-f\\
\overset{拉氏变换}\Rightarrow sY(s) &= kK_p(Y_{ref}(s) - Y(s))-\frac{f}{s}\\
Y &= \frac{kK_pY_{ref}}{s+kK_p} - \frac{f}{s(s+kK_p)}
\end{aligned}$$

对于阶跃期望值 $Y_{ref} = \dfrac{r}{s}$ ，系统响应为 $\dfrac{kK_pr-f}{(s+kK_p)s}$ ，转化到时域为 $\left(r-\dfrac{f}{kK_p}\right)\left(1-e^{-kK_pt}\right)$ ，稳态误差为 $\dfrac{f}{kK_p}$ ，**必然没有超调**。

### 比例积分控制 PI

我们考虑是否需要引入积分控制。倘若采用 PI 控制，有

$$\begin{aligned}\dot{y} &= k\left( K_p(y_{ref} - y) + K_iD^{-1}(y_{ref} - y) \right)-f\\
\overset{拉氏变换}\Rightarrow s^2Y(s) &= kK_ps(Y_{ref}(s) - Y(s)) + kK_i(Y_{ref}(s)-Y(s))-f\\
Y &= \frac{(kK_ps + kK_i)Y_{ref} - f}{s^2 + kK_ps+kK_i}\\
&= \left(1-\frac{s^2}{s^2+kK_ps+kK_i}\right)Y_{ref} - \frac{f}{s^2+kK_ps+kK_i}
\end{aligned}$$

对于阶跃期望值 $Y_{ref} = \dfrac{r}{s}$ ，系统响应为 $\dfrac{r}{s} - \dfrac{sr-f}{s^2+kK_ps+kK_i}$ ，转化到时域为 $r-\dfrac{r}{\sqrt{k^2K_p^2-4kK_i}}\left(\frac{kK_p+\sqrt{k^2K_p^2-4kK_i}}{2}e^{-\frac{kK_p+\sqrt{k^2K_p^2-4kK_i}}{2}t} - \frac{kK_p-\sqrt{k^2K_p^2-4kK_i}}{2}e^{-\frac{kK_p-\sqrt{k^2K_p^2-4kK_i}}{2}t}\right) -\dfrac{f}{\sqrt{k^2K_p^2-4kK_i}}\left(e^{-\frac{kK_p+\sqrt{k^2K_p^2-4kK_i}}{2}t} - e^{-\frac{kK_p-\sqrt{k^2K_p^2-4kK_i}}{2}t}\right) $ ，稳态误差为 0 。

注意，以上形式的解仅适用于 $K_p^2>4K_i$ 的情况。**倘若 $K_i$ 相比于 $K_p$ 过大，过渡过程会出现超调乃至高频减幅振荡**。

### 比例微分控制 PD

我们考虑是否需要引入微分控制。倘若采用 PD 控制，有

$$\begin{aligned}\dot{y} &= k\left( K_p(y_{ref} - y) + K_dD(y_{ref} - y) \right)-f\\
\overset{拉氏变换}\Rightarrow sY(s) &= kK_p(Y_{ref}(s) - Y(s)) + kK_ds(Y_{ref}(s)-Y(s))-\frac{f}{s}\\
Y &= \frac{(kK_ds + kK_p)Y_{ref} - f/s}{s(1+kK_d)+kK_p}\\
&= \left(1-\frac{s}{s(1+kK_d)+kK_p}\right)Y_{ref}-\frac{f}{s\left(s(1+kK_d)+kK_p\right)}
\end{aligned}$$

对于阶跃期望值 $Y_{ref} = \dfrac{r}{s}$ ，系统响应为 $\dfrac{r}{s} - \dfrac{r}{s(1+kK_d)+kK_p}-\dfrac{f}{s\left(s(1+kK_d)+kK_p\right)}$ ，转化到时域为 $r-\dfrac{f}{kK_p} + (\dfrac{f}{kK_p}-\dfrac{r}{1+kK_d})e^{-\dfrac{kK_p}{1+kK_d}t}$ ，稳态误差为 $\dfrac{f}{kK_p}$ 。当 $\dfrac{f}{kK_p}-\dfrac{r}{1+kK_d} > 0$ 时有超调，反之没有超调。但系统输出总小于给定值。引入微分控制后，系统瞬时响应更快，调节时间变长。

![Figure_2.png](./系统阶数与%20PID.assets/Figure_2.png)

1. 如果控制器的前馈补偿足够好，使得扰动项 $f = 0$ ，那么我们不考虑 PI 控制。
2. 如果控制器的前馈补偿一般，扰动项存在，但是很不明显，那么我们一般也不考虑 PI 控制。
3. 如果要求高控制精度，选用 PI 控制（要小心过渡过程的震荡风险！）。
4. 如果给定值变化频率较快，要求高响应速度，选用 PD 控制。
5. 如果对控制器要求不高，只用 P 控制就能满足大部分工况。

## 一阶系统串级控制

在这一小节，我们来分析一下，针对一阶系统，使用串级控制方案能否获得更好的控制效果。

### 内外环比例控制 P+P

一阶系统是 $\dot{y} = ku$ ，我们先简单一点，内外双环都用比例控制，也就是说

$$\begin{aligned}
\left(\dot{y}\right)_{ref} &= K_{p1}(y_{ref} - y)\\
\dot{y} &= kK_{p2}(\left(\dot{y}\right)_{ref} - \dot{y})-f\\
&= kK_{p2}(K_{p1}(y_{ref}-y)-\dot{y})-f\\
\overset{拉氏变换}\Rightarrow sY(s) &= kK_{p1}K_{p2}Y_{ref}(s)-kK_{p1}K_{p2}Y(s)-kK_{p2}sY(s)-\frac{f}{s}\\
Y &= \frac{kK_{p1}K_{p2}Y_{ref}-f/s}{(1+kK_{p2})s+kK_{p1}K_{p2}}
\end{aligned}$$

这也是个一阶惯性环节，对于阶跃期望值 $Y_{ref} = \dfrac{r}{s}$ ，系统响应为 $\dfrac{kK_{p1}K_{p2}r-f}{\left[(1+kK_{p2})s+kK_{p1}K_{p2}\right]s}$ ，转化到时域为 $\dfrac{kK_{p1}K_{p2}r-f}{kK_{p1}K_{p2}}\left(1-e^{-\frac{kK_{p1}K_{p2}}{1+kK_{p2}}t}\right)$ ，稳态误差为 $\dfrac{kK_{p1}K_{p2}r-f}{kK_{p1}K_{p2}}$ ，必然没有超调。

控制效果与一阶系统直接比例控制基本等价。

### 外环比例控制，内环比例积分控制 P+PI

之所以想到内环用 PI ，是因为上文已经证明零阶系统用 PI 控制效果最好。

> Q：积分项会引入系统响应的“滞后”，为什么我们可以在内环引入微分项？
> A：我们引入积分项，是为了让内环误差更快缩减至容许范围内；至于误差要花更长时间变为 0 ，内环要花更长时间才能稳定，在某些情况下我们是可以接受的。

此时有

$$\begin{aligned}
\left(\dot{y}\right)_{ref} &= K_{p1}(y_{ref} - y)\\
\dot{y} &= kK_{p2}(\left(\dot{y}\right)_{ref} - \dot{y}) + kK_{i}D^{-1}(\left(\dot{y}\right)_{ref} - \dot{y})-f\\
&= kK_{p2}(K_{p1}(y_{ref}-y)-\dot{y}) + kK_{i}D^{-1}(K_{p1}(y_{ref}-y)-\dot{y})-f\\
\overset{拉氏变换}\Rightarrow s^2Y(s) &= kK_{p1}K_{p2}sY_{ref}(s)-kK_{p1}K_{p2}sY(s)-kK_{p2}s^2Y(s)\\
&\ \ \ + kK_iK_{p1}Y_{ref}(s)-kK_iK_{p1}Y(s)-kK_isY(s)-f\\
Y &= \frac{(kK_{p1}K_{p2}s + kK_iK_{p1})Y_{ref}-f}{(1+kK_{p2})s^2+k(K_{p1}K_{p2}+K_i)s+kK_iK_{p1}}\\
&= \frac{kK_{p1}K_{p2}}{1+kK_{p2}}\frac{(s+K_i/K_{p2})Y_{ref}-f/(kK_{p1}K_{p2})}{s^2+\frac{k(K_{p1}K_{p2}+K_i)}{1+kK_{p2}}s+\frac{kK_iK_{p1}}{1+kK_{p2}}}
\end{aligned}$$

对于阶跃期望值 $Y_{ref} = \dfrac{r}{s}$ ，系统响应为 $\frac{kK_{p1}K_{p2}}{1+kK_{p2}}\frac{(r-f/(kK_{p1}K_{p2}))s+rK_i/K_{p2}}{s(s^2+\frac{k(K_{p1}K_{p2}+K_i)}{1+kK_{p2}}s+\frac{kK_iK_{p1}}{1+kK_{p2}})}$ ，转化到时域的表达式有点复杂，此处略去。起始点是 0 ，稳态误差为 0 ，内环运行过程中的误差值减小，系统动态性能有所改善。

### 外环比例微分控制，内环比例积分控制 PD+PI

推导太繁琐，懒得写了，直接给结论：

起始点位于 $(0,r)$ 之间，稳态误差为 0 ，瞬态响应增加，调节时间稍有增长。

### 小结

综上所述，对于一阶系统，使用串级控制方案是可行的，如果参数调得足够好，的确能获得更好的控制效果。但相较于单环纯比例控制 + 较好的前馈，其控制效果提升不明显。因此这样做的**性价比一般**。

在工程实际中，需要思考：是否能获得 $\dot{y}$ 的反馈信号？如果能，其准确性、噪声如何？（此处但凡有一个答案是否定的，我们都不应首先考虑串级控制方案。）

此外，还有一些与 PID 本身相关的工程问题：
1. 以上分析都局限于阶跃输入情况。当给定值频繁变化时，微分项会放大这一变化，导致系统输出频繁波动，甚至引发振荡。为了避免这一问题，可以只对反馈信号做微分，称为“微分先行”。
2. 微分项会放大反馈信号的噪声，对控制造成干扰。为解决这一问题，可以考虑对微分项输出做滤波。
3. 内环的积分项会引入“滞后”，需要斟酌是否能接受。如果遇到了积分饱和现象，这一“滞后”就更加严重。


## 二阶系统

参见 [云台 PID 理论分析](云台%20PID%20理论分析.md)
