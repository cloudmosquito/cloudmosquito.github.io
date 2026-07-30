# ROS2 手册

> 目标：做成一份随查随用的参考手册，覆盖从概念与版本、安装与环境配置、工作空间（colcon/ament）、节点开发（rclpy/rclcpp）、通信机制（Topic/Service/Action/Parameters/Lifecycle）、DDS 与 QoS、工具链（ros2 CLI / ros2 launch / ros2 bag2 / rviz2 / gazebo/ignition）、常见包（nav2 / moveit2 / ros2_control）、调试与性能调优、安全（SROS2）和跨版本迁移（ROS1 <-> ROS2）。

---

## 目录

1. ROS2 概览与发行版

2. 安装与环境配置（binary / source / keyring 注意事项）

3. 工作空间与构建系统（colcon / ament）

4. 包结构、消息定义与接口生成

5. 节点编写：rclpy（Python）与 rclcpp（C++）实战

6. 通信模型：Topic / Service / Action / Parameters / Parameter Events

7. 生命周期节点（Managed Nodes）与 Executors

8. DDS 与 QoS（详细参数与常见配置）

9. Composition（组件化）与多节点进程

10. Launch 系统（ros2 launch — 基于 Python）

11. ros2 bag2、logging、rqt 与可视化工具

12. TF2 与坐标变换

13. 仿真：Gazebo / Ignition / ROS2 插件

14. 常见中间件、包与栈：nav2 / moveit2 / ros2_control / image_pipeline

15. 跨版本互通：ros1_bridge、消息迁移策略

16. SROS2（安全）基础

17. 调试、测试与持续集成（单元测试、集成测试）

18. 部署建议、性能优化与多机器人网络配置

19. 常见问题与排错清单

20. 附录：代码模板、CMake/Package 示例、QoS 範例、常用命令速查表

---

## 1. ROS2 概览与发行版

**ROS 2** 是面向现代机器人系统的重构版本，基于 DDS（Data Distribution Service）实现通信中间件，支持实时/分布式/多平台（Linux/Windows/macOS）等特性。

发行版遵循一年一发的节奏（通常在 5 月 23 日左右发布），偶数年发布 LTS（长期支持）。常见发行：Foxy、Galactic、Humble、Iron、Jazzy、Kilted（2025）。


> 注意：你若需要选择用于研发/部署的发行版，推荐以 LTS 作为服务器/长期项目基线（例如 Jazzy 为 2024 发布的 LTS 系列），而短期发行更适合试新特性或升级测试。

---

## 2. 安装与环境配置

### 2.1 官方二进制安装（Ubuntu 为例）

官方一般提供针对常见 Ubuntu 版本（如 20.04/22.04/24.04）的二进制包；安装步骤大致：

1. 配置 apt 源（添加 Open Robotics 的 apt 仓库与 keyring）。

2. 安装基础软件包（`ros-<distro>-desktop`、`ros-<distro>-desktop-full` 或按需选择）。

3. 初始化依赖：`sudo apt update`，安装 `python3-colcon-common-extensions` 等构建工具。

### 2.2 源码编译（从 ros2 / 相关仓库构建）

在需要定制或使用未发布包时，常从源码编译。用到的工具：`vcs`、`colcon`、`rosdep`。

基本流程：

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
vcs import src < ros2.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 2.3 重要注意：签名密钥与源的迁移（2025 以后可能需要）

自 2025 年中起，ROS 的 apt signing key 曾进行更新（如果你在 2025 年之前配置了老的 key，可能需要替换），遇到 `apt update` 错误时请确认你的 `ros-archive-keyring.gpg` 并参考官方迁移指南。

### 2.4 跨平台注意

ROS2 在 Windows/macOS 上也有支持（但包覆盖和第三方依赖差异较大）。开发建议在主力目标平台上优先测试（例如部署到 Ubuntu 22.04/24.04 时在相同系统上开发）。

---

## 3. 工作空间与构建系统（colcon / ament）

### 3.1 package 格式与元数据

ROS2 通常使用 `package.xml` + `CMakeLists.txt`（`ament` 系列的 helper 宏），也支持 `ament_python` 对于纯 Python 包。

### 3.2 colcon 常用命令

- 构建：`colcon build`（推荐加 `--symlink-install` 便于开发）
    
- 清理：`colcon build --clean` 或删除 `build/ install/ log/`
    
- 指定包构建：`colcon build --packages-select pkg_name`
    
- 并行、日志：`colcon build --parallel-workers 8`，`--event-handlers console_direct+`。

---

## 4. 包结构、消息定义与接口生成

`.msg` / `.srv` / `.action` 文件与 ROS1 类似，但生成与编译通过 `rosidl` 系统与 `rosidl_default_generators` 完成。

在 `package.xml` 和 `CMakeLists.txt` 中声明 `rosidl_default_generators`、`rosidl_default_runtime` 等依赖以启用消息代码生成。

---

## 5. 节点编写：rclpy（Python）与 rclcpp（C++）实战

### 5.1 rclpy 简单 Publisher（Python）

```python
# talker_rclpy.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.count = 0
    def timer_callback(self):
        msg = String()
        msg.data = f'hello {self.count}'
        self.publisher_.publish(msg)
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 5.2 rclcpp 简单 Publisher（C++）

```cpp
// talker_rclcpp.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("talker_cpp");
  auto publisher = node->create_publisher<std_msgs::msg::String>("topic", 10);
  rclcpp::WallRate loop_rate(10);
  int count = 0;
  while (rclcpp::ok()) {
    auto msg = std_msgs::msg::String();
    msg.data = "hello " + std::to_string(count++);
    publisher->publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
```

---

## 6. 通信模型（Topic / Service / Action / Parameters）

### 6.1 Topic（发布/订阅）

使用 DDS 作为底层传输，Topic 行为受 QoS 配置影响。

常用 CLI：`ros2 topic list` / `ros2 topic echo /topic` / `ros2 topic pub /topic std_msgs/String "data: 'hi'"`

### 6.2 Service（同步调用）

使用 `create_service` / `create_client`。

 CLI：`ros2 service list` / `ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "a: 1\nb: 2"`

### 6.3 Action（长时任务）

ROS2 的 action api 与 ROS1 类似，但基于 `action_msgs` 与 `rclcpp_action` / `rclpy.action` 包。

CLI：`ros2 action list` / `ros2 action send_goal /action_name some_action_type "{goal: 1}"`

### 6.4 参数与参数事件

ROS2 params 是节点级别的，支持参数事件订阅（当参数被改变时，可触发回调），并支持参数描述（min/max/type）用于生命周期和动态 reconfigure 的替代。

 API：`declare_parameter`, `get_parameter`, `add_on_set_parameters_callback`。
 
---

## 7. 生命周期节点（Managed Nodes）与 Executors

Managed Node（lifecycle node）允许节点在明确的生命周期状态（unconfigured, inactive, active, finalized）中切换，适合对启动/停止/安全行为有严格要求的系统（例如机器人控制栈）。

Executors 管理回调执行策略：`SingleThreadedExecutor`, `MultiThreadedExecutor`，可以自定义回调组和优先级策略以提高吞吐和实时性。

---

## 8. DDS 与 QoS（详细）

### 8.1 常见 QoS 策略

- **Reliability**：`RELIABLE` / `BEST_EFFORT`。
    
- **Durability**：`VOLATILE` / `TRANSIENT_LOCAL`。
    
- **History**：`KEEP_LAST(n)` / `KEEP_ALL`。
    
- **Depth**：历史深度（和 KEEP_LAST 联动）。

- **Deadline / Lifespan / Liveliness**：用于更严格的时间约束和生存性检测。

### 8.2 实战建议

对关键控制命令使用 `RELIABLE + KEEP_LAST(1)`。

对高频传感器（如 LiDAR 点云）使用 `BEST_EFFORT` 减少传输延迟和重试开销（如果丢包可接受）。

在跨网络或多主机部署时，注意各主机 DDS 实现（Fast-RTPS / CycloneDDS / RTI Connext）对性能与选项的差异。

---

## 9. Composition（组件化）与多节点进程

使用组件（component nodes）可以把多个节点以插件方式加载到同一进程，降低进程间通信开销（避免 DDS 复制），提升性能与内存占用效率。

常用 API：`rclcpp_components`。

---

## 10. Launch 系统（ros2 launch）

Launch 文件使用 Python API 编写，支持条件、参数传递、节点组合、事件处理等。

示例：

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    node = Node(package='my_pkg', executable='talker', name='talker')
    ld.add_action(node)
    return ld
```

---

## 11. ros2 bag2、logging、rqt 与可视化工具

- `ros2 bag`（bag2）用于记录与回放数据，支持不同的 storage 后端（sqlite3, zstd, etc.）。
    
- `rqt` 工具与 rviz2 可用于调试与可视化。`ros2 run rviz2 rviz2`。

---

## 12. TF2 与坐标变换

ROS2 使用 `tf2`，API 与 ROS1 类似，但需注意时间管理与 Buffer 的使用。使用 `tf2_ros` 发布/监听变换。
 
---

## 13. 仿真：Gazebo / Ignition（现在称为 Ignition / Citadel / Fort? 取决于发行）

ROS2 生态在不断迁移到 Ignition（ignition gazebo / gazebo-ros2 pkgs）或将 Gazebo Classic 与 Ignition 之间的桥接。选择与 ROS2 发行版本相兼容的 Gazebo/ignition 版本并使用相应的 ROS2 插件。

---

## 14. 常见中间件与栈：nav2 / moveit2 / ros2_control

- **nav2**：ROS2 的导航栈（基于 BT 路径规划、行为树、地图与成本地图）。
    
- **moveit2**：机械臂运动规划在 ROS2 上的实现（与 MoveIt 1 类似但 API 有差异）。
    
- **ros2_control**：硬件抽象与控制器接口，结合 `ros2_controllers` 提供 PID、JointTrajectory 等控制器。

---

## 15. 跨版本互通：ros1_bridge / 迁移策略

- `ros1_bridge` 提供 ROS1 <-> ROS2 的桥接（话题、服务、action），两边需对应 message 类型。
    
- 迁移建议：逐步把节点迁移到 ROS2，保留桥接直到所有节点完成迁移，优先迁移基础中间件和安全关键路径。

---

## 16. SROS2（安全）基础

SROS2 提供基于 DDS Security 的安全配置（加密、认证、访问控制）。生产部署时建议启用（复杂性较高，需要生成并分发安全证书和策略）。

---

## 17. 调试、测试与 CI

- 单元测试：`ros2 pkg create --build-type ament_cmake` 会生成测试框架示例。使用 `ament_cmake` 的 `ament_lint_auto`、`ament_cmake_pytest` 等。
    
- 在 CI 中常用 `colcon test`、`pytest` 与 `ament_lint`。
    
- 对 native 崩溃使用 `gdb`，对 Python 异常查看 `~/.ros/log` 与 `ros2 run` 的标准输出。

---

## 18. 部署建议、性能优化与多机器人网络配置

- 对频繁数据交换的节点使用组件化以减少 DDS 复制开销。
    
- 使用专门的 DDS 实现调优（CycloneDDS 与 Fast-RTPS 常见），并设置合适的参数如线程数、租期、segment size。
    
- 多机器人时推荐设置合适的子网、使用 DDS 局部域（domain_id）隔离不同机器人组的流量。

---

## 19. 常见问题与排错清单

- 节点不能互相发现：检查 `ROS_DOMAIN_ID`、网络配置、DDS 实现策略（multicast 是否开启）。
    
- QoS 不匹配导致不通信：确认双方 QoS 配置（RELIABLE/ BEST_EFFORT，DURABILITY 等）。
    
- 参数不生效：确认是否 `declare_parameter` 并在 launch 中正确传入。

---

## 20. 附录：常用命令速查表

```bash
ros2 topic list / echo / pub / info
ros2 service list / call / info
ros2 action list / send_goal
ros2 node list / info
ros2 param list / get / set
colcon build --symlink-install
ros2 launch pkg file.launch.py
ros2 bag record -a
ros2 bag play
```

---

> 备注：文档内容旨在作为离线参考；针对每个发行版的细节（支持的平台、官方安装步骤、LTS/End-of-life）会随时间变化。若你需要，我可以把手册的“发行版兼容表”和“安装步骤”按你目标 Ubuntu 版本（例如 22.04 或 24.04）精确化。