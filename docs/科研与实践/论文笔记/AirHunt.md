# AirHunt

AirHunt: Bridging VLM Semantics and Continuous  Planning for Efficient Aerial Object Navigation

这篇论文要做的事是 Aerial autonomous target search . VLM 对自然语言指令的理解能力为该领域带来了新的范式。然而，直接将 VLM 视为一个分步骤的离散动作规划器有三个问题：

1. 频率不匹配。VLM 推理延迟通常超过 2000ms，而无人机的实时规划模块频率 >10Hz，两者严重不匹配；当下主要做法是 stop-and-infer，无人机悬停以等待 VLM 推理，无法保持飞行连续性、效率低、对续航不友好。
2. 有限的三维场景理解。VLM 获得的是二维视觉信息，难以将不同观察视角下的同一个物体信息关联在一起并整合到全局地图中，导致动作不稳定，尤其是复杂大规模的户外场景。
3. 缺乏语义指导（semantic guidance）+ 几何运动效率的统一优化框架。过分强调粗糙且不确定的语义指导导致绕路，浪费电池资源；侧重几何运动效率，导致目光短浅，忽视重要区域。

---

设计理念：将 VLM 重新定位为一个 high-level dense semantic generator 而不是一个直接的动作生成器，并且要通过 3D representation 整合语义信息。也就是说，VLM 只用于生成语义先验指导，planner 照常规划路径，两者通过 3D value map 联系在一起。

具体到算法层面，AirHunt 由两个核心模块组成，一个是 Active Dual-Task Reasoning，另一个是 Semantic-Geometric Coherent Planning .

AirHunt 分为三个运行阶段：

1. 原地旋转360°初始化；
2. 初始化后，ADTR 持续更新 3D value map，SGCP 生成探索轨迹；
3. ADTR 确认目标后，系统切换至目标导向导航。

## Active Dual-Task Reasoning

ADTR 选择有足够信息量的关键帧，并且给VLM发指令，执行两个任务：

1. 推断不同区域的语义价值，以更新 3D value map
2. 判断检测到的物体是不是任务目标

---

该模块设计基于以下两个核心假设：

1. VLMs 能够基于形式自由的语言指令，以概率形式直接量化图像区域的语义相关性。直白点说，给VLM一张图片，它能判断某部分“像不像目标”，还能给出一个“有多像”的分数。
2. 室外场景中与任务相关的语义信息（即关键物体和线索）具有高度稀疏性，这使得在保留关键信息的同时，能够对冗余帧进行大幅过滤。

---

该模块自适应地选择图像，构建两种 coverage-aware 和 task-aware 两种不同类型的关键帧，前者用于 maximize geometric coverage of the 探索空间，后者用于捕捉和任务相关的细节信息。

### 1 Active Keyframe Construction

#### 1.1 Coverage-aware Keyframe Construction

**Coverage-aware 关键帧要解决的任务是，从相机高频（>20Hz）拍摄的图像流中尽可能少地选取几帧，同时尽可能多地“重建”环境。**

我们把一个 coverage-aware 关键帧定义为

$$\mathcal{K}_s^i=\left\langle I_i,\mathcal{V}(I_i)\right\rangle$$

其中，$I_i$ 是一张相机拍摄的 RGB 图像和对应的相机姿态； $\mathcal{V}(I_i)\subset \mathbb{R}^3$ 是当前相机姿态下可视的 3D 体素集合，也即这张图像中包含的 3D 体素集合。

> 我们用 3D 体素网格表示环境。

用 $\mathcal{S}_t^{cov}$ 表示截至时刻 t 的 coverage-aware 关键帧集合。

对于一个新的 $I_t$ ，我们将其与 $S_{t-1}^{cov}$ 中的所有关键帧比较，计算其“重叠率”（overlap ratio）：

$$R(I_t,I_i) = \frac{|\mathcal{V}(I_t)\cap\mathcal{V}(I_i)|}{|\mathcal{V}(I_t)|},$$

只有当重叠率小于一定阈值，我们才将其纳入集合。

!!! warning


    我看到这种做法，本能反应是计算复杂度过高，需要下功夫在工程上优化。


#### 1.2 Task-aware Keyframe Construction

**Task-aware 关键帧要解决的任务是，从从相机高频（>20Hz）拍摄的图像流中尽可能少地选取几帧，同时尽可能多地提供任务目标相关信息。**

我们把一个 task-aware 关键帧定义为

$$\mathcal{K}_{e}^{j}=\left\langle I_j,\mathcal{O}(I_j)\right\rangle,$$

其中，$\mathcal{O}(I_j)=\{o_1,o_2,\dots,o_M\}$ 是在图像 $I_j$ 中识别到的目标集合。每个 $o_j$ 都可表示为一个三元组 $(c_j, s_j, \mathbf{x}_j)$ ，$c_j$ 代表目标类别，$s_j\in[0,1]$ 代表识别的置信度，$\mathbf{x}_j\in\mathbb{R}^3$ 代表目标的三维坐标。

> 这里的目标识别、分割和坐标变换，使用的是 [Concept-graphs](https://concept-graphs.github.io/) 方法。

用 $\mathcal{S}_t^{task}$ 表示截至时刻 t 的 task-aware 关键帧集合。

具体而言，给定了一条语言指令，比如 "find a trash bin on the roadside" 之后，我们先让 VLM 输出 K 个与该指令最相关的类别，比如 trash bin、garbege、road side、curb、bench 等等，我们用 $\mathcal{C}_{target}=\{c_1^*,c_2^*,\dots,c_K^*\}$ 表示。

!!! question


    我倒是不怀疑 VLM 能否给出最相关的 K 个类别，毕竟 LLM 的文字生成能力毋庸置疑。但我怀疑 VLM 给出的这 top-K 个类别，真正能用于引导搜索的有多少？会不会绝大部分都是无用的？


对于一个新的 $I_t$ ，只要它的 $\mathcal{O}(I_t)$ 中的类别与 $\mathcal{C}_{target}$ 有重复，我们就将其纳入集合 $\mathcal{S}_t^{task}$ 。

### 2 Asynchronous Dual-Task VLM Reasoning

我们构建的关键帧集合 $\mathcal{S}_t^{cov}$ 和 $\mathcal{S}_t^{task}$ ，其中的关键帧数量达到一定阈值后，就会触发 VLM 推理任务。

任务一：语义价值推断。给定自然语言指令 $\mathcal{L}$ 和关键帧集合 $\mathcal{S}_t^{cov}$ ，对于集合中的每一帧 $\mathcal{K}_{s}^{i}$ ，计算语义价值

$$\mathcal{v}_i=\mathrm{VLM}(\mathcal{K}_{s}^{i},\mathcal{L})\in[0,1].$$

$v_i$ 用于表示关键帧 $\mathcal{K}_{s}^{i}$ 所示区域内包含 $\mathcal{L}$ 中所述目标的概率。

任务二：相关目标验证。给定自然语言指令 $\mathcal{L}$ 和关键帧集合 $\mathcal{S}_t^{task}$ ，搜索集合中包含的任务目标

$$(\mathcal{K}_e^*,o^*)=\mathrm{VLM}(\mathcal{S}_t^{task},\mathcal{L}),$$

其中的 $o^* \in \mathcal{O}(\mathcal{K}_e^*)$ 是识别出的目标对象。

### 3 3D Value Map Composition

我们维护一个 3D value map 来引导无人机在三维空间中的探索。把任务一获得的语义值编码到该地图中。

我们用 $\mathbf{V} \in \mathbb{R}^{W\times H\times D}$ 表示 3D value map ，体素分辨率为 $r$ 。地图包含两个通道：

$$\mathbf{V}=\left(\mathbf{V}^{sem},\mathbf{V}^{conf}\right) ,$$

-  $\mathbf{V}^{sem}\in[0,1]$ 编码能找到目标的可能性，也就是任务一中的“语义价值”；
-  $\mathbf{V}^{conf}\in[0,1]$ 编码置信度。

直观地看，离无人机越远的体素，VLM 判断的准确性越低，我们对其判断结果的置信度也越低。用数学语言描述，对于任意一个可视的，坐标在 $\mathbf{x}_k$ 的体素 $\mathbf{v}_k$ ，其与坐标在 $\mathbf{p}_t$ 的无人机的距离是 $d_k = \left\|\mathbf{x}_k-\mathbf{p}_t\right\|_2$ ，那么该体素的**置信度**就是

$$c_k^t=\max\left(0,1-\frac{d_k}{d_{\max}}\right),$$

其中，$d_\max$ 代表无人机相机的最远感知距离。

每当任务一执行完毕，得到一个关于关键帧 $\mathcal{K}_{s}^{i}$ 的语义价值 $v_i$ 时，我们使用该价值异步更新价值地图。对于所有在 $\mathcal{V}(I_i)$ 里的体素，其 $\mathbf{V}^{sem}$ 更新方式为

$$v_k^t=\frac{c_k^{t-1}\cdot v_k^{t-1}+c_k^t\cdot v_i}{c_k^{t-1}+c_k^t},$$

其 $\mathbf{V}^{conf}$ 更新方式为

$$c_k^t=\frac{(c_k^{t-1})^2+(c_k^t)^2}{c_k^{t-1}+c_k^t}.$$

这种更新方式的优点是：

1. 新旧置信度对称；
2. 高置信度观测主导；
3. 随着探索过程，置信度只增不减；
4. 综合2、3，到探索后期，置信度将基本保持不变。

## Semantic-Geometric Coherent Planning

虽然已经有了一个 3D value map，但由于 VLM 提供的语义先验是稀疏、概率性的、spatially diffuse 的，所以不能直接使用。

严格按照语义价值由高到低探索，容易导致路径振荡；优先考虑几何结构，则导致 VLM 提供的语义信息发挥不出作用。因此，需要特殊算法加以融合、处理。

首先根据 semantic similarity 和 geometric proximity 聚类 frontiers 前沿边界点，然后生成一些能够优先观察高价值区域的观察点。最后，计算出一条全局最优的路径，优先途径 semantic value 高的区域，同时最小化路径长度。

### 1 Semantic-aware Frontier Construction

传统的 frontiers 指的是已知空间和未知空间的边界。传统的前沿探索方法，利用这种结果，通过纯几何距离指标实现探索，不考虑任务相关性。本文提出一种 semantic frontier 语义前沿，把几何信息和语义线索融合在一起，使下游规划器能借助细粒度的语义知识高效探索环境。

我们使用 volumetric occupancy grid map 体积占用网格地图来表示环境，结合上文 3D value map，我们可以为每一个体素 $k$ 赋一个语义价值 $v_k\in[0,1]$ 。

一个语义前沿聚类定义为 $F_i=\langle\mathcal{C}_i,\mathbf{p}_i,s_i \rangle$ ，其中

-  $\mathcal{C}_i=\{c_1,c_2,\dots,c_n\}$ 聚类 $i$ 中的所有前沿体素；
-  $\mathbf{p}_i\in\mathbb{R}^3$ 代表 $\mathcal{C}_i$ 中所有点的几何中心；
-  $s_i = \dfrac{1}{|\mathcal{C}_i|}\sum_{c_j\in\mathcal{C}_i}v(c_j)$ 代表聚类 $i$ 的平均语义价值，$|\mathcal{C}_i|$ 代表 $\mathcal{C}_i$ 中的体素点数量，$v(c_j)$ 代表体素 $c_j$ 的语义价值。

每当传感器完成一次测量更新占用网格地图时，我们都会增量式更新 frontier structure 以保证计算效率。

具体的更新算法借鉴了 An improved seeded region growing algorithm 和 Fuel: Fast uav exploration using incremental frontier structure and hierarchical planning . 创新之处在于引入语义约束。

用 $\mathcal{N}(c)$ 表示体素 $c$ 的上下左右前后 6 个相邻体素集合。在聚类 $F_i$ 根据种子体素 $c_s$ 扩张的过程中，对于聚类 $F_i$ 里的任意一个前沿体素 $c_k\in\mathcal{C}_i$ ，考虑其相邻体素 $c_j\in\mathcal{N}(c_k)$ ，如果它是未被访问过的前沿体素，同时满足

$$|v(c_j)-v(c_s)|<\tau_s,$$

则将其纳入聚类。其中 $\tau_s$ 是预设的阈值。

### 2 Semantic-Geometric Viewpoint Generation

对于每个前沿聚类 $F_i$ ，我们围绕其几何中心 $\mathbf{p}_i$ 生成一组候选的 viewpoints 用于观察未探索到的区域。该方法借鉴了 Racer: Rapid collaborative exploration with a decentralized multi-uav system .

于是，我们得到一个关于前沿聚类 $F_i$ 的候选 viewpoint 集合 $\mathcal{U}_i=\{u^1_i,u^2_i,\dots,u^{M_i}_i\}$ ，每个 viewpoint $u_i^j = (\mathbf{p}_i^j,\theta_i^j)$ ，其中 $\mathbf{p}_i^j$ 为位置，$\theta_i^j$ 为 yaw 角度。

我们要评估每个 viewpoint 的几何覆盖度和语义重要性，以选出最有信息量的一个。

对于每个候选的 viewpoint $u_i^j$ ，在 sensor 的 field of view（FoV）里统计所有未被遮挡的 frontier 体素，该集合记为 $\mathcal{N}_{obs}(u_i^j)$ ，则该 viewpoint 的信息量收益为：

$$S(u_i^j)=\sum_{c_k\in\mathcal{N}_{obs}(u_i^j)}v(c_k).$$

我们把前沿聚类 $F_i$ 的所有候选 viewpoint 中，信息量收益最高的那个记为 $u_i^*$ . 这就是我们最终选定的 viewpoint .

### 3 Optimization with Selective Constraint Injection

接下来，我们的任务是根据以上所得语义前沿聚类和对应的 viewpoints，求解一条全局路径，这条路径应优先考虑目标相关区域，同时最小化飞行时间。

首先，如何结合可能产生矛盾的“目标相关优先”和“飞行时间最短”？思路是先给 viewpoints 划分优先级，语义价值差不多的前沿聚类共享同一优先级。同一优先级内按照最小化飞行时间规划路径，不同优先级按优先级从高到低规划路径先后。

接下来，我们分析一下如何做到“飞行时间最短”。对于任意两个 viewpoint $u_i = (\mathbf{p}_i,\theta_i)$ 和 $u_j = (\mathbf{p}_j,\theta_j)$ ，定义它们之间的几何代价为

$$C(u_i,u_j)=\max\left\{\frac{d(\mathbf{p}_i,\mathbf{p}_j)}{v_{\max}},\frac{|\theta_i-\theta_j|}{\omega_{\max}}\right\},$$

其中，$d(\mathbf{p}_i,\mathbf{p}_j)$ 是在占用栅格图上用 A* 算法求得的路径长度，$v_{\max}$ 和 $\omega_{\max}$ 分别是最大平移速度和最大角速度。

> 我们把 viewpoint 视为节点；把 viewpoints 之间的路径按照语义优先级从高到低连接（同优先级内部不连；相差多个优先级不连），视为有向边；$C(u_i,u_j)$ 即为两节点间有向边的权重。

两个前沿聚类之间的几何代价就是

$$C_{geo}(F_i,F_j)=C(u_i^*,u_j^*).$$

我们构造一个代价矩阵 $\mathbf{C}_G\in\mathbb{R}^{(N_c+1)\times(N_c+1)}$ ，这个代价矩阵包含几何代价和语义约束：

$$\mathbf{C}_G=\begin{bmatrix}c_{c,c} & \mathbf{c}^{\top}_{c,k}\\
\mathbf{c}_{k,c} & \mathbf{C}_{k,k}\end{bmatrix},$$

下标 $c$ 代表当前位置，下标 $k$ 代表前沿聚类。其中，$\mathbf{C}_{k,k}\in\mathbb{R}^{N_c\times N_c}$ ，编码了两个前沿聚类之间的代价，定义为：

$$\mathbf{C}_{k,k}(i,j)=\begin{cases}-1 & \text{if }(i,j)\in\mathcal{P}\text{ and }i\neq j,\\
0 & \text{if }i=j,\\
C_{geo}(F_i,F_j) & \text{otherwise}.
\end{cases}$$










