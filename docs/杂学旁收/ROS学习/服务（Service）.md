# 服务（Service）

## 基本特点

**服务** 是 **请求-响应** 模式的通信机制，主要由客户端和服务器两部分组成。客户端发起一次请求，等待服务器处理并返回响应后继续。

1. 一对一：客户端调用某个指定服务名，服务端监听并响应。（多个客户端同时调用同一个服务时，每个调用都由服务器依次或并发处理；若无服务器，则调用失败或超时）
2. 同步：一旦调用，要等待服务器处理，调用线程阻塞直到收到响应或超时。

适合一次性、明确请求-回复、对延迟容忍度较高、需要确认结果的场景（如获取或设置参数、执行某个动作命令、查询状态快照等），通常调用频率低。

## 自定义服务

以一个服务 AddTwoInts 为例：

### 1 .srv 文件

在某个 ROS 包的 `srv/` 目录下新建一个 `.srv` 文件，文件内容分为请求/响应两部分，例如：

```yaml
int64 a
int64 b
---
int64 sum
```

这表示客户端请求提供两个 `int64` 类型 `a, b`，服务器响应返回一个 `int64` 的 `sum` 。

### 2. CMakeLists.txt 和 package.xml 设置

`package.xml` 中添加依赖：

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

> 其中，`<build_depend>message_generation</build_depend>` 用于在编译时生成 C++ 头文件、Python 模块等。
> `<exec_depend>message_runtime</exec_depend>` 用于在安装或运行时加载并使用已生成的消息/服务代码。

`CMakeLists.txt` 中：

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
## 在这里列出 srv 文件，默认扫描包的 srv/ 目录
add_service_files(
  FILES
  AddTwoInts.srv
)
## 基于 add_service_files 的 .srv 文件生成消息
generate_messages(
  DEPENDENCIES
  std_msgs  # 如果 srv 使用到 std_msgs 中的类型
)
## catkin 包声明
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)
```

> 其中，`message_generation` 是专门用于生成消息/服务的 CMake 模块，提供 `add_service_files, generate_messages` 等宏；
> 
> `message_runtime` 代表运行时需要用到已生成的消息/服务类型；其他包在依赖这个包时，CMake 会将 message_runtime 及这个包的 include 路径自动加入。

### 3. 生成 C++ 头文件和 Python 模块

执行 `catkin_make` 或 `catkin build` 操作后：

catkin 会在 build 目录下生成对应的 C++ 头文件，位置在 `<workspace>/build/your_package/your_package/AddTwoInts.h` ，编译完成后，该头文件安装到 `<workspace>/devel/include/your_package/AddTwoInts.h` 。

此时，工作空间内的所有 C++ 代码中只需要 `#include "your_package/AddTwoInts.h"` 或 `#include <your_package/AddTwoInts.h>` ，并且 CMakeLists.h 中 `include_directories(${catkin_INCLUDE_DIRS})` 指定了 devel/include ，就能找到该服务的头文件。

同时，catkin 会生成 Python 模块，安装到

```swift
<workspace>/devel/lib/python3/dist-packages/your_package/srv/_AddTwoInts.py
<workspace>/devel/lib/python3/dist-packages/your_package/srv/AddTwoInts.py
```

执行 `source <workspace>/devel/setup.bash` 后，就可以在 Python 脚本中通过 `from your_package.srv import AddTwoInts, AddTwoIntsRequest` 使用服务。

### 服务端（Server）实现

```c++
#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"  // 由 AddTwoInts.srv 生成

// 回调函数：当有客户端调用时被触发
bool add(beginner_tutorials::AddTwoInts::Request &req,
         beginner_tutorials::AddTwoInts::Response &res)
{
  // 处理请求：例如相加两个整数
  res.sum = req.a + req.b;
  ROS_INFO("request: a=%ld, b=%ld", (long)req.a, (long)req.b);
  ROS_INFO("sending response: %ld", (long)res.sum);
  return true;  // 返回 true 表示成功响应；若返回 false，则客户端收到失败
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle nh;

  // advertiseService: 服务名 "add_two_ints"，回调函数 add
  ros::ServiceServer service = nh.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();  // 保持服务端运行，处理请求回调
  return 0;
}
```

`advertiseService` 需要两个参数，第一个为服务名（可以是绝对名或相对名，可通过 ROS namespace 机制管理）；第二个为回调函数或可调用对象，签名必须为 `bool callback(RequestType&, ResponseType&)` 。

回调函数收到 Request，填充 Response，返回值 bool 表示自定义逻辑层面是否成功，若返回 false，服务器仍会发送响应。客户端 call() 返回 false 代表调用失败，通常只在服务器无法正常处理时出现。

### 客户端（Client）实现

```c++
#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  ros::NodeHandle nh;

  // 创建 ServiceClient，类型 AddTwoInts，服务名 "add_two_ints"
  ros::ServiceClient client = nh.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");

  // 等待服务可用（可选，但推荐），超时可自行处理
  if (!client.waitForExistence(ros::Duration(5.0))) {
    ROS_ERROR("Service add_two_ints not available after waiting");
    return 1;
  }

  beginner_tutorials::AddTwoInts srv;
  srv.request.a = 3;
  srv.request.b = 4;
  if (client.call(srv)) {
    ROS_INFO("Sum: %ld", (long)srv.response.sum);
  } else {
    ROS_ERROR("Failed to call service add_two_ints");
  }
  return 0;
}
```

#### 客户端初始化

`serviceClient<ServiceType>("service_name")` 对应于模板函数

```c++
ServiceClient serviceClient(const std::string& service_name, bool persistent=false);
```

返回一个 `ros::ServiceClient` 服务客户端对象。其中，service_name 对应要连接的服务名称，可以是相对名、全局名或私有名。

persistent 表示是否建立 TCP 通信持久连接，默认为 false，也就是每次 client.call(srv) 的时候建立一次 TCP 连接，服务完成后断开；若设为 true，则只在第一次建立连接，后续复用此连接。

非持久连接能自动适应服务端重启、网络异常等情况，但开销更大。

#### 客户端调用服务

`client.call(srv)` 是同步阻塞调用，返回 `true` 表示通信成功并成功收到响应。但不代表业务成功，还需检查 `srv.response` 字段

可以先调用 `waitForExistence()` 等待服务出现，避免直接 `call()` 立即失败。

> `client.waitForExistence(ros::Duration(5.0))` 代表最多等待服务器 5 s ，若服务器 5 s 后还没出现，返回 false；反之，若服务器在 5 s 内出现，则返回 true 。
