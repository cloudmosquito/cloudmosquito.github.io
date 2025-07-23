# PX4 降落移动平台

2025.06.21

## 目的

探索是否能完全依赖 PX4 飞控原生功能，实现移动平台异地起降。

## 结论

就四旋翼仿真实验而言，PX4 飞控支持上位机通过 MAVROS 协议实时更新降落点位置，能跟踪以 0.6-0.8 m/s （接近实际情况）速度运动的移动平台并降落。

不足之处，该方案高度依赖：1. 平台报告的经纬度精度及报告频率；2. 飞机自身的经纬度精度；特别是平台的报告频率。就仿真而言，降落误差为米级。

## 细节

仿真采用PX4自带的 iris 四旋翼无人机模型，用 QGC 地面站预先规划航迹：起飞（黄色T点）——途经（2号点）——降落（绿色L点）。我们假设L点为移动平台初始位置，并假设平台以经纬度均为0.000005 度/秒的速度移动（约0.6-0.8 m/s），实际运行效果如下所示：

![](./PX4降落移动平台.assets/QGC效果.png){.img-center width=100%}

下图分别是：降落时经度期望值与反馈值（单位：度）；纬度期望值与反馈值（单位：度）；高度期望值与反馈值（单位：米）。

![](./PX4降落移动平台.assets/降落时的经纬度和高度.png){.img-center width=100%}

其中，x, y 方向降落过程中的误差值均为 0.5 m 左右。

下图分别是 x,y,z 三个方向降落时的速度（单位：米/秒）

![](./PX4降落移动平台.assets/降落时的速度.png){.img-center width=100%}

跟随降落过程中，四旋翼水平方向移动速度约为 0.63 m/s 。

## 附录-实现方法

主要利用了 mavros 的如下内容：

![](./PX4降落移动平台.assets/mavros截图.png){.img-center width=100%}

订阅 mavros/mission/waypoints 话题，以获得 mavros_msgs:: WaypointList 类型消息，该消息按顺序存储了当前任务的航点数据。

通过 mavros/mission/push 服务，修改航点列表中的最后一个点的数据，并将其推送给飞控。

```cpp
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointPush.h>

ros::ServiceClient wp_push_client;
mavros_msgs::WaypointList current_wps;

void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	if (current_wps.waypoints.empty()) {
		return;
	}
	
	auto& land_wp = current_wps.waypoints.back();
	land_wp.x_lat = msg->pose.position.x;
	land_wp.y_long = msg->pose.position.y;
	land_wp.z_alt = msg->pose.position.z;
	
	mavros_msgs::WaypointPush push_srv;
	push_srv.request.waypoints = current_wps.waypoints;
	if (wp_push_client.call(push_srv)) {
		ROS_INFO("Landing point updated to (%.6f, %.6f, %.2f)", 
			land_wp.x_lat, land_wp.y_long, land_wp.z_alt);
	} else {
		ROS_ERROR("Waypoint update failed");
	}
}

void wpListCallback(const mavros_msgs::WaypointList::Constptr& msg)
{
	current_wps = *msg;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dynamic_landing_node");
	ros::NodeHandle nh;
	
	wp_push_client = nh.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
	
	ros::Subscriber wp_list_sub = nh.subscribe("mavros/mission/waypoints", 1, wpListCallback);
	ros::Subscriber target_sub = nh.subscribe("/dynamic_landing_target", 10, targetCallback);
	
	ros::spin();
	return 0;
}
```

## 多线程（2025.07.23 更新）

### 问题描述

由于有对平台位置进行预测/外推的需求，因此代码中新增订阅了 `/mavros/global_position/global` 话题，其回调函数类似：

```cpp
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	// 一些逻辑
	a[0] = msg->latitude;
	a[1] = msg->longitude;
}
```

但在 CT 飞机的仿真联调测试时发现，数组 a 里的两个数据呈现明显的“阶梯状”，即有较长一段时间，数据保持不变——这显然是不合理的。

最终定位问题为 `targetCallback()` 回调函数里的 `wp_push_client.call(push_srv)` 服务调用导致主线程堵塞。由于我们的代码默认只有一个线程，因此 `gpsCallback()` 回调函数也被堵塞了。

实测在 `targetCallback()` 中发布的话题频率最高只能到 3 Hz，而 `targetCallback()` 订阅的话题频率是 80 Hz！

### 解决方法 1

我们的解决方法是：

```c++
#include <thread>

std::mutex srv_mutex_;

void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	// 一些逻辑
    auto waypoints_copy = current_wps_.waypoints;
    std::thread(&DynamicLandingNode::callServiceInThread, this, waypoints_copy).detach(); // 这里用 this 是因为这两个函数都被我们放到了同一个类里
}

void callServiceInThread(const std::vector<mavros_msgs::Waypoint>& waypoints)
    {
        std::lock_guard<std::mutex> lock(srv_mutex_);  // 加锁，保证同一时刻只有一个线程调用服务
        mavros_msgs::WaypointPush push_srv_;
        push_srv_.request.waypoints = current_wps_.waypoints;
        if (wp_push_client_.call(push_srv_)) {
            ROS_INFO("Successfully updated waypoints");
        } else {
            ROS_WARN("Failte to update waypoints.");
        }
    }
```

### 解决方法 2

将 `main` 函数中的

```c++
ros::spin();
```

改成多线程的 

```c++
ros::AsyncSpinner spinner(4) // 开四个线程，由ROS管理
spinner.start();
ros::waitForShutdown();
```

也能解决 `targetCallback()` 堵塞影响 `gpsCallback()` 的问题。但是无法解决 `targetCallback()` 中发布话题频率过低的问题。