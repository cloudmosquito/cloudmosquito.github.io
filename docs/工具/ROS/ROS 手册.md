# ROS 手册

> 目标：成为一份随查随用的参考手册，覆盖从环境搭建、工作空间与包管理、编写节点（C++/Python）、通信机制（Topic/Service/Action/Parameter）、重要工具（roscore/rosrun/roslaunch/rostopic/rosnode/rosparam/rosbag/rqt/tf）、调试与常见问题、以及进阶主题（插件、控制回路、仿真、性能调优等）。

---

## 目录

1. 简介与概念速览
2. 安装与环境配置
3. Catkin 工作空间与包管理
4. 消息、服务与动作（Topic / Service / Action）
5. 节点：rospy 与 roscpp 实战
6. Launch 文件与参数（roslaunch / rosparam）
7. TF（坐标变换）
8. rosbag 与回放
9. 调试工具与诊断
10. 仿真：Gazebo 与 RViz
11. 常见任务示例（移动机器人、机械臂、视觉）
12. 性能优化与发布最佳实践
13. 常见问题与排查流程
14. 附录：CMakeLists.xml / package.xml / 常用 message/service 生成

---

## 1. 简介与概念速览

- **ROS（Robot Operating System）1**：分布式通信框架，提供节点（process）、Topic（发布/订阅）、Service（同步RPC）、Action（带反馈的长时任务）、Parameter Server 等。
- **Master**：注册节点、名称解析（roscore 启动）。
- **节点（node）**：最小可运行单元，可用 C++（roscpp）或 Python（rospy）实现。
- **包（package）**：代码组织单位，包含 `package.xml`, `CMakeLists.txt`。
- **工作空间（catkin workspace）**：编译、管理包的目录结构（`src/`、`build/`、`devel/`、`install/`）。

---

## 2. 安装与环境配置（概览）

### 常见安装步骤（Ubuntu 系列）

1. 初始化 rosdep：
    
```bash
sudo rosdep init # 第一次安装 ROS 时执行一次
rosdep update    # 更新依赖数据库，最好每隔一段时间执行一次
```

rosdep 是 ROS 提供的一个系统依赖管理工具。第一步是初始化 rosdep 的数据库配置，在 `/etc/ros/rosdep/sources.list.d/` 下生成一个配置文件，告诉 rosdep 以后要去哪查找依赖。第二步是从网络上下载并更新依赖数据库缓存。执行后，rosdep 会去 GitHub 的 `ros/rosdistro` 仓库中拉取最新的依赖规则文件。

假设我们下载了一个 ROS 包，可以如下为其安装系统级依赖：

```bash
rosdep install --from-paths src --ignore-src -r -y # 实际安装依赖
```

2. 创建并配置工作空间：
    
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 重要提示

- 每次打开新终端，记得 `source /opt/ros/<distro>/setup.bash` 与你的工作空间 `source ~/catkin_ws/devel/setup.bash`。
    
- 建议在 `~/.bashrc` 添加 `source /opt/ros/<distro>/setup.bash`，并根据需要添加工作空间的 source。
    

---

## 3. Catkin 工作空间与包管理

### 创建包

```bash
cd ~/catkin_ws/src
catkin_create_pkg my_pkg roscpp rospy std_msgs geometry_msgs
cd ~/catkin_ws
catkin_make
```

- `package.xml`：元数据（依赖、维护者、许可）。
    
- `CMakeLists.txt`：编译规则。常见需要在 `find_package(catkin REQUIRED COMPONENTS ...)` 添加依赖，设置 `catkin_package()`，并用 `add_executable()`、`target_link_libraries()`。
    

### 编译与安装

- `catkin_make`：简单，适合单一工作空间。
    
- `catkin build`（来自 `catkin_tools`）：更灵活，支持并行构建、选包构建、安装空间。
    
- 清理：`catkin_make clean` 或手动删除 `build/ devel/`。
    

---

## 4. 消息、服务与动作

### Topic（发布/订阅）

- Publisher 与 Subscriber 的生命周期与 QoS（ROS1 没有 QoS 控制像 ROS2）。
    
- 示例（Python 发布者）：
    
```python
# talker.py
import rospy
from std_msgs.msg import String

rospy.init_node('talker')
pub = rospy.Publisher('chatter', String, queue_size=10)
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    msg = "hello %s" % rospy.get_time()
    pub.publish(msg)
    rate.sleep()
```

- 查看 topic：`rostopic list`；检查消息：`rostopic echo /chatter`；查看带宽：`rostopic bw /chatter`。
    

### Service（RPC）

- Server 与 Client；同步调用。
    
```python
# service_server.py
from beginner_tutorials.srv import AddTwoInts, AddTwoIntsResponse
import rospy

def handle_add(req):
    return AddTwoIntsResponse(req.a + req.b)

rospy.init_node('add_two_ints_server')
s = rospy.Service('add_two_ints', AddTwoInts, handle_add)
rospy.spin()
```

调用：`rosservice call /add_two_ints 1 2` 或在客户端代码中调用。

### Action（actionlib）

- 用于长时任务（带预empt 与反馈），比如导航、抓取。包含 `Goal`、`Result`、`Feedback`。
    
- 使用 `actionlib.SimpleActionServer` 与 `actionlib.SimpleActionClient`。
    
---

## 5. 节点：rospy 与 roscpp 实战

### roscpp（C++）快速样例

`src/talker.cpp`：

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "talker_cpp");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 10);
  ros::Rate r(10);
  while (ros::ok()){
    std_msgs::String msg;
    msg.data = "hello from c++";
    pub.publish(msg);
    r.sleep();
  }
}
```

`CMakeLists.txt` 中添加：

```cmake
add_executable(talker_cpp src/talker.cpp)
target_link_libraries(talker_cpp ${catkin_LIBRARIES})
```

### rospy（Python）快速样例

- 见上方 Topic 示例。
    
- packaging: 在 `scripts/` 下放可执行 Python 文件，并 `chmod +x`，CMakeLists 中 `catkin_install_python(PROGRAMS scripts/talker.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})`

### nodelet

常见的 node 是一个节点一个进程，而 nodelet 作为一个插件式的 ROS 组件，与 node 最大的不同是：**多个 nodelet 可以在同一个 nodelet manager 进程中运行** 。

这种设计是为了减少同一数据在不同节点间跨进程通信带来的拷贝和序列化性能开销。

普通 node（cpp） 通过在 CMakeLists.txt 里写 `add_executable(……)` 来生成可执行文件；而 nodelet（cpp）通过 `add_library(……)` 生成 .so 动态库，由 nodelet 管理器加载。

```cmake
add_library(SO3Control src/SO3Control.cpp)                   # 核心算法库
add_library(so3_control_nodelet src/so3_control_nodelet.cpp) # nodelet 动态库

target_link_libraries(so3_control_nodelet 
    ${catkin_LIBRARIES}
    SO3Control                                               # 链接核心算法
)
```

nodelet 代码编写时应注意：
1. 继承 `nodelet::Notelet` 类
2. 必须实现 `onInit()` 函数，该函数相当于常见 node 中的 `main()` 函数。

---

## 6. Launch 文件与参数（roslaunch / rosparam）

### 基本 Launch 文件

```xml
<launch>
  <node pkg="my_pkg" type="talker.py" name="talker" output="screen">
    <param name="greeting" value="hi"/>
  </node>
</launch>
```

启动：`roslaunch my_pkg my_launch.launch`。

### Launch 文件里的参数含义

在 ROS 的 `.launch` 文件中，`<node>` 标签用于启动一个可执行程序（节点），`name`、`pkg`、`type`、`output`、`args` 是该标签最常用的属性。

假设你要在终端运行一个程序：
```bash
rosrun rviz rviz -d config.rviz
```
对应的 launch 文件写法就是：
```xml
<node pkg="rviz" type="rviz" name="my_rviz" output="screen" args="-d config.rviz" />
```

下面是每个参数的详细含义：

#### 1. `pkg` (Package)
*   **含义**：**功能包名称**。
*   **作用**：告诉 ROS 系统，这个可执行程序位于哪个功能包（Package）里。ROS 会根据这个名字去 `src` 目录下查找对应的包。
*   **示例**：`pkg="rviz"` 表示去 `rviz` 这个功能包里找。
*   **类比**：就像你去文件夹里找软件，`pkg` 是**文件夹的名字**。

#### 2. `type`
*   **含义**：**可执行文件名称**。
*   **作用**：指定要运行的具体程序（二进制文件或脚本）的名字。通常与编译后的文件名一致（例如 C++ 的 `add_two_ints_server` 或 Python 的 `listener.py`）。
*   **示例**：`type="rviz"` 表示运行名为 `rviz` 的可执行文件。
*   **类比**：这是**软件的可执行文件名**（比如 `chrome`, `python`, `ls`）。
*   **注意**：`pkg` 和 `type` 结合起来，就等同于 `rosrun <pkg> <type>`。

#### 3. `name`
*   **含义**：**节点名称 (Node Name)**。
*   **作用**：这是节点在 ROS 系统运行时的**唯一标识符**。ROS 内部的通信（话题、服务、参数）都基于这个名字。
*   **示例**：`name="rvizvisualisation"`。运行后，使用 `rosnode list` 会看到 `/rvizvisualisation`。
*   **重要性**：
    *   如果多个节点用同一个 `name`，后启动的会把先启动的“踢下线”（默认行为）。
    *   日志文件也会以这个名字命名，方便排查问题。
*   **类比**：这是程序运行起来后起的**昵称**。比如你运行微信，进程名可能是 `WeChat`，但在系统里它显示为“微信”。

#### 4. `output`
*   **含义**：**输出方式**。
*   **作用**：决定节点的打印信息（如 `printf`, `rospy.loginfo`, `ROS_INFO`）显示在哪里。
*   **常用值**：
    *   **`"screen"`**：输出直接打印在**终端屏幕**上。
        *   *用途*：调试时使用，可以实时看到报错或日志。
    *   **`"log"`**：输出保存到**日志文件**中（通常在 `~/.ros/log/` 下），终端不显示。
        *   *用途*：正式运行时使用，保持终端干净，或者后台运行的节点。
*   **示例**：`output="log"` 表示日志存文件，不刷屏。

#### 5. `args` (Arguments)
*   **含义**：**命令行参数**。
*   **作用**：传递给可执行程序的额外参数。
*   **示例**：`args="-d $(find exploration_manager)/config/swarm.rviz"`。
    *   这相当于在终端运行 `rviz -d ...`。
    *   这里使用了 `$(find ...)` 语法，用于动态获取路径。
*   **用途**：常用于加载配置文件、设置初始参数、开启特定模式等。
*   **类比**：就像 `ls -l` 中的 `-l`，或者 `python script.py --port 8080` 中的 `--port 8080`。

---

还有几个非常常用的参数，建议一并了解：

| 参数 | 含义 | 示例 | 作用 |
| :--- | :--- | :--- | :--- |
| **`respawn`** | 自动重启 | `respawn="true"` | 如果节点崩溃或退出，**自动重新启动它**。常用于关键节点（如驱动、定位）。 |
| **`ns`** | 命名空间 | `ns="/robot1"` | 给节点加前缀。例如名字变为 `/robot1/rvizvisualisation`。常用于多机器人系统。 |
| **`cwd`** | 工作目录 | `cwd="node"` | 设置节点运行时的当前工作目录（默认是 `ROS_HOME`）。 |
| **`launch-prefix`** | 启动前缀 | `launch-prefix="gnome-terminal -e"` | 在运行节点前加命令。常用于**单独弹出一个终端窗口**来调试某个节点。 |

#### 总结示例

把上面所有概念结合起来，一个完整的节点启动描述如下：

```xml
<node 
    pkg="exploration_manager"          <!-- 1. 去哪个包里找 -->
    type="explorer_node"               <!-- 2. 运行哪个程序 -->
    name="main_explorer"               <!-- 3. 在 ROS 里叫什么名字 -->
    output="screen"                    <!-- 4. 日志打印到终端 -->
    args="--config default.yaml"       <!-- 5. 传给程序的参数 -->
    respawn="true"                     <!-- 6. 挂了自动重启 -->
    ns="/robot_01"                     <!-- 7. 放在 /robot_01 命名空间下 -->
/>
```

理解这些参数后，你就可以随意修改或编写自己的 `.launch` 文件来管理复杂的机器人系统了。

### 参数服务器

- 使用 `rosparam set /namespace/param value`，在运行中读取 `rospy.get_param('~param', default)` 或 `nh.getParam("/param", var)`。
    
- 可以把参数写成 YAML 并在 launch 中加载：
    
```xml
<rosparam file="$(find my_pkg)/config/config.yaml" command="load"/>
```

---

## 7. TF（坐标变换）

- `tf`（旧）与 `tf2`（推荐）。用于管理多个坐标系之间的转换。
    
- 常用 API：`tf::TransformListener`（C++）或 `tf.TransformListener()`（Python）。发布静态变换：`static_transform_publisher`。
    
- 关键点：设置适当的父/子坐标系、频率、时间戳，对重复广播的 frame_id 保持稳定。
    

示例（Python 监听）：

```python
import rospy
import tf
rospy.init_node('tf_listener')
listener = tf.TransformListener()
try:
    listener.waitForTransform('/base_link', '/camera_link', rospy.Time(), rospy.Duration(4.0))
    (trans, rot) = listener.lookupTransform('/base_link', '/camera_link', rospy.Time(0))
except Exception as e:
    rospy.logerr(e)
```

---

## 8. rosbag 与回放

- 记录：`rosbag record -a` 或 `rosbag record /topic1 /topic2`。
    
- 回放：`rosbag play file.bag`，可加参数 `--pause`, `-r <rate>`。
    
- 检查信息：`rosbag info file.bag`。
    
- 使用 Python API 读取 bag 文件：`rosbag.Bag('file.bag')`。
    

---

## 9. 调试工具与诊断

- `roscore`：确保运行。
    
- `rosnode list` / `rosnode info /node`。
    
- `rostopic list` / `rostopic echo /rostopic hz /rostopic pub`。
    
- `rosservice list` / `rosservice call`。
    
- `rqt_graph`：查看节点/主题连接图。
    
- `rqt_console` 与 `rosconsole`：日志过滤与级别（DEBUG/INFO/WARN/ERROR/FATAL）。
    
- 使用 `gdb` 或 `rosrun rqt_reconfigure` 等工具排查 native 崩溃与参数问题。
    

调试流程建议：

1. 确认 `roscore` 在运行。
    
2. `rosnode list` 看你的节点是否已注册。
    
3. `rqt_graph` 检查连接拓扑。
    
4. `rostopic echo` 检查数据流。
    
5. 若 C++ 崩溃，使用 `gdb` 运行节点：`rosrun --prefix 'gdb -ex run --args' pkg node`。
    

---

## 10. 仿真：Gazebo 与 RViz

- **RViz**：可视化传感器、坐标系、路径、Marker。常见问题：frame_id 不对、时间戳不一致导致显示延迟。
    
- **Gazebo**：物理仿真，与 ROS 通过 `gazebo_ros_pkgs` 集成。常见工作流：在 URDF/SDF 中定义模型，使用 `ros_control` + `ros_controllers` 控制。
    

---

## 11. 常见任务示例

### 移动机器人（简化）

- 订阅里程计与激光，发布 `cmd_vel`。
    
- 使用 `move_base`（navigation stack）完成路径规划（需要 `costmap`, `global_planner`, `local_planner`）。
    
### 机械臂

- 使用 `moveit` 进行运动规划：创建 SRDF、设置规划组、配置规划管线（OMPL）。
    
### 视觉处理

- 使用 `image_transport` 传输图像，`cv_bridge` 转换 OpenCV 图像。
    
---

## 12. 性能优化与发布最佳实践

- 避免高频率下过多的 ROS Topic 小消息，合并或降低频率。
    
- 对重计算使用多线程或节点拆分，利用 `queue_size` 控制内存。
    
- 使用 `latching`（对一次性消息）和合适的 `queue_size`。
    
- 对大量数据（点云、图像）采用压缩或 `image_transport` 的 compressed 插件。
    
---

## 13. 常见问题与排查流程

- **节点未注册到 Master**：确认 `roscore` 在运行、网络设置（`ROS_MASTER_URI`、`ROS_HOSTNAME`/`ROS_IP`）正确。
    
- **主题无消息**：用 `rostopic hz` 检查频率，`rostopic echo` 验证消息内容。
    
- **TF 报错：Lookup would require extrapolation into the future**：时间同步问题，检查发布时间戳，使用 `rospy.Time(0)` 或延时查询。
    
- **包找不到**：确保 `source devel/setup.bash`，并检查 `package.xml` 依赖是否已安装。
    
---

## 14. 附录：常用模板与 CMakeLists 片段

### package.xml 最小模板

```xml
<package format="2">
  <name>my_pkg</name>
  <version>0.0.0</version>
  <description>Example package</description>
  <maintainer email="you@example.com">You</maintainer>
  <license>BSD</license>
  <depend>roscpp</depend>
  <depend>rospy</depend>
  <depend>std_msgs</depend>
</package>
```

### CMakeLists 基本骨架

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(my_pkg)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
add_executable(talker_cpp src/talker.cpp)
target_link_libraries(talker_cpp ${catkin_LIBRARIES})
```

---

> 结束语：本手册为可扩展的基础参考，覆盖了日常绝大多数 ROS1 开发与调试场景。你可以告诉我需要加强哪一章（例如更详细的 MoveIt 教程、网络多机部署、实时控制、或特定传感器驱动），我会立即把对应内容加入到同一文档中。