# 系统阶数与 PID

我们分别讨论以下三个系统的 PID 控制器设计原则：
  
$$\begin{aligned}
y_1 &= k_1u_1\\
\dot{y}_2 &= k_2u_2\\
\ddot{y}_3 &= k_3u_3\\
\end{aligned}$$

其中， $y$ 是系统输出，可以被测量； $u$ 是控制输入； $k$ 是常系数。

## 零阶系统

### 比例控制 P

针对零阶系统 $y = ku$ ，倘若我们单纯采用比例控制，那么有

$$\begin{aligned}y &= k\cdot K_p(y_{ref} - y)\\
\overset{拉氏变换}\Rightarrow Y(s) &= kK_p(Y_{ref}(s) - Y(s))\\
\frac{Y}{Y_{ref}} &= \frac{kK_p}{1+kK_p} 
\end{aligned}$$

对于阶跃期望值 $Y_{ref} = \dfrac{r}{s}$ ，系统响应为 $\dfrac{kK_pr}{(1+kK_p)s}$ ，转化到时域为 $\dfrac{kK_pr}{1+kK_p}$ ，是一个有稳态误差的阶跃值。

![Figure_1.png](./系统阶数与%20PID.assets/Figure_1.png)

## 比例积分控制 PI

为了消除这一稳态误差，我们需要采用 PI 控制，有

$$\begin{aligned}y &= k\left(K_p(y_{ref} - y) + K_iD^{-1}(y_{ref}-y)\right)\\
\overset{拉氏变换}\Rightarrow sY(s) &= kK_ps(Y_{ref}(s) - Y(s)) + kK_i(Y_{ref}(s)-Y(s))\\
\frac{Y}{Y_{ref}} &= \frac{kK_ps + kK_i}{s+kK_ps+kK_i} = 1-\frac{s}{s+kK_ps+kK_i} 
\end{aligned}$$

对于阶跃期望值 $Y_{ref} = \dfrac{r}{s}$ ，系统响应为 $\dfrac{r}{s} - \dfrac{r}{s+kK_ps+kK_i}$ ，转化到时域就是 $r-\dfrac{r}{1+kK_p}e^{-\frac{kK_i}{1+kK_p}t}$ 。稳态误差是 0 。

在此基础上，我们思考一下是否需要引入微分控制。倘若采用 PID 控制，有

$$\begin{aligned}y &= k\left( K_p(y_{ref} - y) + K_iD^{-1}(y_{ref}-y) + K_dD(y_{ref}-y) \right)\\
\overset{拉氏变换}\Rightarrow sY(s) &= kK_ps(Y_{ref}(s) - Y(s)) + kK_i(Y_{ref}(s)-Y(s)) + kK_ds^2(Y_{ref}(s)-Y(s))\\
\frac{Y}{Y_{ref}} &= \frac{kK_ds^2+kK_ps + kK_i}{kK_ds^2+(1+kK_p)s+kK_i} = 1-\frac{s}{kK_ds^2+(1+kK_p)s+kK_i} 
\end{aligned}$$

对于阶跃期望值 $Y_{ref} = \dfrac{r}{s}$ ，系统响应为 $\dfrac{r}{s} - \dfrac{r}{kK_ds^2+(1+kK_p)s+kK_i}$ ，转化到时域就是 $r-\dfrac{r}{\sqrt{(1+kK_p)^2-4k^2K_iK_d}}e^{-\dfrac{1+kK_p}{2kK_d}t}\left(e^{\dfrac{\sqrt{(1+kK_p)^2-4k^2K_iK_d}}{2kK_d}t} - e^{-\dfrac{\sqrt{(1+kK_p)^2-4k^2K_iK_d}}{2kK_d}t}\right)$ ，稳态误差是 0 ，但是有超调。

![Figure_3.png](./系统阶数与%20PID.assets/Figure_3.png)

所以，零阶系统从理论上最好用 PI 控制。

## 一阶系统

### 比例控制 P

针对一阶系统 $\dot{y} = ku$ ，倘若我们单纯采用比例控制，那么有

$$\begin{aligned}\dot{y} &= k\cdot K_p(y_{ref} - y)\\
\overset{拉氏变换}\Rightarrow sY(s) &= kK_p(Y_{ref}(s) - Y(s))\\
\frac{Y}{Y_{ref}} &= \frac{kK_p}{s+kK_p} 
\end{aligned}$$

这是一个一阶惯性环节。对于阶跃期望值 $Y_{ref} = \dfrac{r}{s}$ ，系统响应为 $\dfrac{kK_pr}{(s+kK_p)s}$ ，转化到时域为 $r\left(1-e^{-kK_pt}\right)$ ，稳态误差为 0 。

### 比例微分控制 PD

既然稳态误差已经是 0 了，我们自然没有必要考虑引入积分控制。我们考虑一下是否需要引入微分控制。倘若采用 PD 控制，有

$$\begin{aligned}\dot{y} &= k\left( K_p(y_{ref} - y) + K_dD(y_{ref} - y) \right)\\
\overset{拉氏变换}\Rightarrow sY(s) &= kK_p(Y_{ref}(s) - Y(s)) + kK_ds(Y_{ref}(s)-Y(s))\\
\frac{Y}{Y_{ref}} &= \frac{kK_ds + kK_p}{s(1+kK_d)+kK_p}  = 1-\frac{s}{s(1+kK_d)+kK_p}
\end{aligned}$$

对于阶跃期望值 $Y_{ref} = \dfrac{r}{s}$ ，系统响应为 $\dfrac{r}{s} - \dfrac{r}{s(1+kK_d)+kK_p}$ ，转化到时域为 $r - \dfrac{r}{1+kK_d}e^{-\dfrac{kK_p}{1+kK_d}t}$ ，稳态误差为 0 。加了微分控制之后，系统瞬时响应更快，调节时间变长。

![Figure_4.png](./系统阶数与%20PID.assets/Figure_4.png)

所以，一阶系统从理论上最好用 P 控制。

## 一阶系统串级控制

在这一小节，我们来分析一下，针对一阶系统，使用串级控制方案能否获得更好的控制效果。

### 内外环比例控制 P+P

一阶系统是 $\dot{y} = ku$ ，我们先简单一点，内外双环都用比例控制，也就是说

$$\begin{aligned}
\left(\dot{y}\right)_{ref} &= K_{p1}(y_{ref} - y)\\
\dot{y} &= kK_{p2}(\left(\dot{y}\right)_{ref} - \dot{y})\\
&= kK_{p2}(K_{p1}(y_{ref}-y)-\dot{y})\\
\overset{拉氏变换}\Rightarrow sY(s) &= kK_{p1}K_{p2}Y_{ref}(s)-kK_{p1}K_{p2}Y(s)-kK_{p2}sY(s)\\
\frac{Y}{Y_{ref}} &= \frac{kK_{p1}K_{p2}}{(1+kK_{p2})s+kK_{p1}K_{p2}}
\end{aligned}$$

这也是个一阶惯性环节，对于阶跃期望值 $Y_{ref} = \dfrac{r}{s}$ ，系统响应为 $\dfrac{kK_{p1}K_{p2}r}{\left[(1+kK_{p2})s+kK_{p1}K_{p2}\right]s}$ ，转化到时域为 $r\left(1-e^{-\frac{kK_{p1}K_{p2}}{1+kK_{p2}}t}\right)$ ，稳态误差为 0 。

控制效果与一阶系统直接比例控制基本等价。

### 外环比例控制，内环比例积分控制 P+PI

之所以想到内环用 PI ，是因为上文已经证明零阶系统用 PI 控制效果最好。此时有

$$\begin{aligned}
\left(\dot{y}\right)_{ref} &= K_{p1}(y_{ref} - y)\\
\dot{y} &= kK_{p2}(\left(\dot{y}\right)_{ref} - \dot{y}) + kK_{i}D^{-1}(\left(\dot{y}\right)_{ref} - \dot{y})\\
&= kK_{p2}(K_{p1}(y_{ref}-y)-\dot{y}) + kK_{i}D^{-1}(K_{p1}(y_{ref}-y)-\dot{y})\\
\overset{拉氏变换}\Rightarrow s^2Y(s) &= kK_{p1}K_{p2}sY_{ref}(s)-kK_{p1}K_{p2}sY(s)-kK_{p2}s^2Y(s)\\
&\ \ \ + kK_iK_{p1}Y_{ref}(s)-kK_iK_{p1}Y(s)-kK_isY(s)\\
\frac{Y}{Y_{ref}} &= \frac{kK_{p1}K_{p2}s + kK_iK_{p1}}{(1+kK_{p2})s^2+k(K_{p1}K_{p2}+K_i)s+kK_iK_{p1}}\\
&= \frac{kK_{p1}K_{p2}}{1+kK_{p2}}\frac{s+K_i/K_{p2}}{s^2+\frac{k(K_{p1}K_{p2}+K_i)}{1+kK_{p2}}s+\frac{kK_iK_{p1}}{1+kK_{p2}}}
\end{aligned}$$

对于阶跃期望值 $Y_{ref} = \dfrac{r}{s}$ ，系统响应为 $r\frac{kK_{p1}K_{p2}}{1+kK_{p2}}\frac{s+K_i/K_{p2}}{s(s^2+\frac{k(K_{p1}K_{p2}+K_i)}{1+kK_{p2}}s+\frac{kK_iK_{p1}}{1+kK_{p2}})}$ ，转化到时域的表达式有点复杂，此处略去。起始点是 0 ，稳态误差为 0 ，其动态性能有所改善。

### 外环比例微分控制，内环比例积分控制 PD+PI

推导有点繁琐，懒得抄了，直接给结论：系统的阶跃响应为

$$\frac{r}{s}-\frac{(\frac{1}{k}+K_p+1)rs}{(\frac{1}{k}+K_p+1+K_pK_d)s^2+(K_pK_{p1}+K_iK_d)s+K_iK_{p1}}$$

起始点位于 $(0,r)$ 之间，稳态误差为 0 ，调节时间稍有增长。

### 小结

综上所述，对于一阶系统，使用串级控制方案是可行的，能获得更好的控制效果。但这样做的性价比不高。

## 二阶系统

参见 [云台 PID 理论分析](云台%20PID%20理论分析.md)
