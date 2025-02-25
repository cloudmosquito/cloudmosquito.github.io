# ROS2 学习记录

## 1 工作空间与功能包

CLI（Command Line Interface， 命令行）

```shell
# 指令意义：启动 <package_num> 包下的 <executable_name> 节点
ros2 run <package_num> <executable_name>

# 查看节点列表
ros2 node list

# 查看节电信息
ros2 node info <node_name>

# 重映射节点名称
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle

使用 ros2 node <command> -h 可以获得更多使用细节
```

工作空间：一个工作空间下可以包含若干个功能包，每个功能包里都可以包含若干个节点。

ROS2 中，功能包根据编译方式的不同分为三类：ament_python, cmake, ament_cmake 。

每个功能包都有一个标配的 manifest.xml 文件，用于记录这个包的名字，构建工具，编译信息，拥有者，作用等信息。通过该信息，就可以自动为该功能包安装依赖，构建时确定编译顺序。

```shell
# 1 安装获取功能包
sudo apt install ros-<version>-package_name
# 安装获取会自动放置到系统目录，不用再次手动 source

# 2 创建功能包
ros2 pkg create <package-name>  --build-type {cmake,ament_cmake,ament_python}  --dependencies <依赖名字>

# 3 列出可执行文件
ros2 pkg executables # 列出所有
ros2 pkg executables <package_num> # 列出某个功能包的

# 4 列出所有功能包
ros2 pkg list

# 5 输出某个包所在路径的前缀
ros2 pkg prefix <package-name>

# 6 列出功能包的清单描述文件
ros2 pkg xml <package-name>

使用 ros2 pkg <command> -h 可以获得更多使用细节
```

---

## 2 Colcon

Colcon 是一个功能包构建工具，可以用来编译工作空间。（ROS2 默认居然是没有安装 Colcon 的！

```shell
# 只编译一个包
colcon build --packages-select <package_name>

# 不编译测试单元
colcon build --packages-select <package_name> --cmake-args -DBUILD_TESTING=0

# 运行编译的包的测试
colcon test

# 允许通过更改 src 下的部分文件来改变 install
# （每次调整 python 脚本的时候就不用重新 build 了）
colcon build --symlink-install

```

详细讲解一下 `--symlink-install` 的作用：

Colcon 编译生成的可执行文件、库文件、配置文件等输出到 build 和 install 目录。`--symlink-install` 会用 符号链接（Symbolic Link）代替直接复制文件到 install 目录。

对于非编译型文件（比如 Python 脚本，配置文件），install 目录中的文件是源码的符号连接，在 src 更改后就可以直接生效，不用再 `colcon build` 了；

对于编译型文件（比如 C++ 可执行文件），install 目录中的是 build 目录中的编译结果，修改了 C++ 源代码之后仍然需要 `colcon build`。

---

## 3 RCL (ROS Client Library，ROS 客户端库) 

在编写功能包以及其中的节点的时候，不管是 c++ 的 rclcpp 库，还是 python 的 rclpy 库，我们的代码总要去包含或者调用 rcl 开头的库函数，这个 rcl 是什么意思呢？

RCL 是 ROS 的一种 API (Application Programming Interface，应用程序编程接口)。

<figure markdown>
![](./ROS2 学习记录.assets/rcl_layer.png){width=100%}
<!-- <figcaption>RCL Layers</figcaption> -->
</figure>

这张图里，最底层是第三方的 DDS（Data Distribution Service 数据分发服务），rmw（ros2 middleware 中间件接口）是对各家 DDS 的统一抽象，基于 rmw 实现了 rclc（给 C 语言用的 API 库）。由于 C 语言是绝大部分语言的鼻祖，因此大部分语言的 API 库都可以基于 rclc 开发，于是就有了 rclcpp，rclpy，rcljava……
