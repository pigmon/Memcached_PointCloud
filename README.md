# Memcached_PointCloud

## 基本功能
* memc_cloud: 将从 velodyne master 中得到的点云信息存储至 Memcached.
* memc_client: 读取 memcached 中的点云数据，并转换成 ros 的 sensor_msgs::PointCloud2

## TODO List
* 目前基础数据是从 PointCloud2 的消息中得到的，正式使用应从 velodyne master 中直接获取，避免一次消息接收。
* 优化读取过程的占用时间
* 添加读取至 pcl 格式的接口

## Bug List
* Point Fields 目前是硬编码（第一个版本的遗漏）
* 目前的 Test Case 发送的消息，在rviz中打开会显示 status error，但点云可以正常显示

## 客户端使用说明
### 基本过程
将 memc_client 中的 mc_reader.h/.cpp 放入工程，并参考 memc_client 中的 CMake 和 Package 添加memcached支持。
### CMake & Package.xml
参考memc_client 中的 CMake 和 Package，增加 memcached 的内容。（假设该工程已经支持 velodyne 相关内容）

#### Package.xml
添加对 memcached 的依赖
```xml
  <build_depend>memcached</build_depend>
  <build_export_depend>memcached</build_export_depend>
  <exec_depend>memcached</exec_depend>
```
CMakeLists
```cmake
link_libraries(memcached)
```
### 接口调用
```c++
#include "mc_reader.h"
...
	sensor_msgs::PointCloud2 msg;

	if (!ReadPointCloud_AsRosMsg(msg))
	{
		std::cout << "Test Case Fail.\n";
		return;
	}
```
具体参考 memc_client 项目中调用的 TestCase 函数
