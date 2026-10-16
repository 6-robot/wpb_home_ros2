# wpb_home_msgs

启智机器人的 ROS2 消息接口包，仅定义消息，无需启动节点。

## 物体检测消息

`wpb_home_msgs/msg/Object` 用于 `/wpb_home/objects_3d` 话题：

```text
std_msgs/Header header
string[] name
float64[] x
float64[] y
float64[] z
float64[] probability
```

数组中相同下标表示同一个物体。`header` 表示时间戳和坐标系，`name` 为名称，
`x/y/z` 为物体参考点坐标（米），`probability` 为置信度。
实物检测节点使用 `base_footprint` 坐标系，参考点为包围框的 x 最大值、
y 中点和 z 最小值，与跟踪时使用的几何中心不同。

C++ 引用：

```cpp
#include <wpb_home_msgs/msg/object.hpp>
// 消息类型：wpb_home_msgs::msg::Object
```

Python 引用：

```python
from wpb_home_msgs.msg import Object
```

使用方在 `package.xml` 中添加 `<depend>wpb_home_msgs</depend>`，
在 CMake 中添加 `find_package(wpb_home_msgs REQUIRED)`，并将
`wpb_home_msgs` 加入对应目标的 `ament_target_dependencies()`。

## 从仿真消息迁移

字段沿用 `wpr_simulation2/msg/Object`，话题名称和 QoS 保持不变。
类型名称已变化，使用旧类型的节点不能直接与新类型通信。
更新后需要同时重新编译并重启实物检测、抓取及其自定义订阅节点。
实物侧不再需要安装 `wpr_simulation2`；仿真工程继续使用自身的消息类型。
