# Mockamap - ROS2 随机地图生成器

[![ROS2](https://img.shields.io/badge/ROS2-Humble%2FIron%2FRolling-blue.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

这是一个用于 ROS2 的随机地图生成器包，可以从 ROS1 迁移而来。它可以生成多种类型的 3D 点云地图，用于机器人导航和路径规划算法的测试。

## 特性

- **4 种地图生成模式**：
  1. **Perlin 噪声 3D** - 使用 Perlin 噪声算法生成自然地形
  2. **随机障碍物** - 随机生成长方体障碍物
  3. **2D 迷宫** - 递归分割算法生成 2D 迷宫
  4. **3D 迷宫** - 基于 Voronoi 图的 3D 迷宫

- **完全兼容 ROS2**：使用 rclcpp 和 ROS2 消息类型
- **可配置参数**：通过 launch 文件或参数服务器自定义地图属性
- **实时发布**：地图以 `sensor_msgs/PointCloud2` 消息格式持续发布

## 依赖

- ROS2 (Humble/Iron/Rolling)
- rclcpp
- sensor_msgs
- PCL (Point Cloud Library)
- Eigen3

## 安装

```bash
cd ~/ros2_ws/src
git clone <repository_url>
cd ~/ros2_ws
colcon build --packages-select mockamap
source install/setup.bash
```

## 使用方法

### 启动默认地图

```bash
ros2 launch mockamap mockamap.launch.py
```

### 在 RViz2 中查看

```bash
ros2 run rviz2 rviz2
```

添加 `PointCloud2` 显示，设置话题为 `/mock_map`。

## 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `seed` | int | 511 | 随机种子，用于复现相同地图 |
| `update_freq` | double | 1.0 | 地图发布频率 (Hz) |
| `resolution` | double | 0.1 | 地图分辨率 (米/体素) |
| `x_length` | int | 10 | X 轴长度 (米) |
| `y_length` | int | 10 | Y 轴长度 (米) |
| `z_length` | int | 3 | Z 轴高度 (米) |
| `type` | int | 1 | 地图类型 (1-4) |

### 类型 1: Perlin 噪声 3D 参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `complexity` | double | 0.03 | 噪声频率，越大越复杂 (0.0~0.5) |
| `fill` | double | 0.3 | 填充比例 (0.0~1.0) |
| `fractal` | int | 1 | 分层层数，越多细节越丰富 |
| `attenuation` | double | 0.1 | 分形衰减系数 (0.0~0.5) |

### 类型 2: 随机障碍物参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `width_min` | double | 0.6 | 障碍物最小宽度 (米) |
| `width_max` | double | 1.5 | 障碍物最大宽度 (米) |
| `obstacle_number` | int | 50 | 障碍物数量 |

### 类型 3: 2D 迷宫参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `road_width` | double | 0.5 | 道路宽度 (米) |
| `add_wall_x` | int | 0 | 是否添加 X 方向边界墙 (0/1) |
| `add_wall_y` | int | 1 | 是否添加 Y 方向边界墙 (0/1) |
| `maze_type` | int | 1 | 迷宫类型 (1=递归分割) |

### 类型 4: 3D 迷宫参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `numNodes` | int | 40 | 节点数量 |
| `connectivity` | double | 0.8 | 连通性系数 (0.0~1.0) |
| `nodeRad` | int | 1 | 节点半径 |
| `roadRad` | int | 10 | 道路半径 |

## 自定义 Launch 示例

```python
# my_map.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mockamap',
            executable='mockamap_node',
            name='mockamap_node',
            output='screen',
            parameters=[{
                'seed': 1234,
                'type': 2,  # 随机障碍物
                'resolution': 0.2,
                'x_length': 50,
                'y_length': 50,
                'z_length': 5,
                'obstacle_number': 30,
                'width_min': 1.0,
                'width_max': 3.0,
            }]
        ),
    ])
```

运行自定义 launch：
```bash
ros2 launch mockamap my_map.launch.py
```

## 地图类型示例

### Type 1: Perlin 噪声 3D
适合模拟自然地形，如山丘、洞穴等。

![Perlin 3D](images/perlin3d.png)

### Type 2: 随机障碍物
适合测试避障算法，障碍物随机分布在空间中。

### Type 3: 2D 迷宫
使用递归分割算法生成经典迷宫结构，可配置道路宽度和边界墙。

![2D Maze](images/post2d.png)

### Type 4: 3D 迷宫
基于 Voronoi 图生成的三维迷宫结构，适合测试无人机路径规划。

## 节点信息

### 发布的话题

- `/mock_map` (`sensor_msgs/msg/PointCloud2`): 生成的地图点云

### 参数

所有参数均在节点启动时通过参数服务器配置，支持动态重配置。

## 与其他 ROS2 包集成

### 与 OctoMap 集成

```bash
ros2 run octomap_server octomap_server_node
# 将 mock_map 话题重映射为 octomap 输入
```

### 与 Navigation2 集成

在 Nav2 配置中设置 `voxel_layer` 或 `obstacle_layer` 订阅 `/mock_map` 话题。

## 常见问题

### Q: 编译时找不到 PCL？
确保已安装 PCL 开发库：
```bash
sudo apt install libpcl-dev
```

### Q: 地图没有显示？
检查 RViz2 中：
1. Fixed Frame 设置为 `map`
2. PointCloud2 话题设置为 `/mock_map`
3. 调整 Decay Time 或增大 Point Size

### Q: 如何保存生成的地图？
可以使用 pcl_ros 的 pointcloud_to_pcd 节点保存：
```bash
ros2 run pcl_ros pointcloud_to_pcd --ros-args --remap input:=/mock_map
```

## 许可证

GPLv3 - 详见 [LICENSE](LICENSE) 文件

## 作者

William Wu

## 致谢

基于原始 ROS1 版本的 mockamap 包迁移而来。
