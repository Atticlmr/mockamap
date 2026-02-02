# Mockamap - ROS2 Random Map Generator

[![ROS2](https://img.shields.io/badge/ROS2-Humble%2FIron%2FRolling-blue.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

A random map generator package for ROS2, migrated from ROS1. It can generate various types of 3D point cloud maps for testing robot navigation and path planning algorithms.

## Features

- **4 Map Generation Modes**:
  1. **Perlin Noise 3D** - Generate natural terrain using Perlin noise algorithm
  2. **Random Obstacles** - Randomly generate cuboid obstacles
  3. **2D Maze** - Generate 2D mazes using recursive division algorithm
  4. **3D Maze** - Generate 3D mazes based on Voronoi diagrams

- **Fully ROS2 Compatible**: Uses rclcpp and ROS2 message types
- **Configurable Parameters**: Customize map properties via launch files or parameter server
- **Real-time Publishing**: Maps are continuously published as `sensor_msgs/PointCloud2` messages

## Dependencies

- ROS2 (Humble/Iron/Rolling)
- rclcpp
- sensor_msgs
- PCL (Point Cloud Library)
- Eigen3

## Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/Atticlmr/mockamap
cd ~/ros2_ws
colcon build --packages-select mockamap
source install/setup.bash
```

## Usage

### Launch Default Map

```bash
ros2 launch mockamap mockamap.launch.py
```

### View in RViz2

```bash
ros2 run rviz2 rviz2
```

Add a `PointCloud2` display and set the topic to `/mock_map`.

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `seed` | int | 511 | Random seed for reproducing the same map |
| `update_freq` | double | 1.0 | Map publishing frequency (Hz) |
| `resolution` | double | 0.1 | Map resolution (meters/voxel) |
| `x_length` | int | 10 | X-axis length (meters) |
| `y_length` | int | 10 | Y-axis length (meters) |
| `z_length` | int | 3 | Z-axis height (meters) |
| `type` | int | 1 | Map type (1-4) |

### Type 1: Perlin Noise 3D Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `complexity` | double | 0.03 | Noise frequency, higher = more complex (0.0~0.5) |
| `fill` | double | 0.3 | Fill ratio (0.0~1.0) |
| `fractal` | int | 1 | Number of fractal layers, more = more detail |
| `attenuation` | double | 0.1 | Fractal attenuation coefficient (0.0~0.5) |

### Type 2: Random Obstacles Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `width_min` | double | 0.6 | Minimum obstacle width (meters) |
| `width_max` | double | 1.5 | Maximum obstacle width (meters) |
| `obstacle_number` | int | 50 | Number of obstacles |

### Type 3: 2D Maze Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `road_width` | double | 0.5 | Road width (meters) |
| `add_wall_x` | int | 0 | Whether to add X-direction boundary walls (0/1) |
| `add_wall_y` | int | 1 | Whether to add Y-direction boundary walls (0/1) |
| `maze_type` | int | 1 | Maze type (1=recursive division) |

### Type 4: 3D Maze Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `numNodes` | int | 40 | Number of nodes |
| `connectivity` | double | 0.8 | Connectivity coefficient (0.0~1.0) |
| `nodeRad` | int | 1 | Node radius |
| `roadRad` | int | 10 | Road radius |

## Custom Launch Example

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
                'type': 2,  # Random obstacles
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

Run custom launch:
```bash
ros2 launch mockamap my_map.launch.py
```

## Map Type Examples

### Type 1: Perlin Noise 3D
Suitable for simulating natural terrain such as hills and caves.

![Perlin 3D](images/perlin3d.png)

### Type 2: Random Obstacles
Suitable for testing obstacle avoidance algorithms with randomly distributed obstacles.

### Type 3: 2D Maze
Generates classic maze structures using recursive division algorithm, with configurable road width and boundary walls.

![2D Maze](images/post2d.png)

### Type 4: 3D Maze
Generates 3D maze structures based on Voronoi diagrams, suitable for testing UAV path planning.

## Node Information

### Published Topics

- `/mock_map` (`sensor_msgs/msg/PointCloud2`): Generated map point cloud

### Parameters

All parameters are configured via the parameter server at node startup, supporting dynamic reconfiguration.

## Integration with Other ROS2 Packages

### Integration with OctoMap

```bash
ros2 run octomap_server octomap_server_node
# Remap mock_map topic as octomap input
```

### Integration with Navigation2

Configure `voxel_layer` or `obstacle_layer` in Nav2 to subscribe to the `/mock_map` topic.

## FAQ

### Q: PCL not found during compilation?
Ensure PCL development libraries are installed:
```bash
sudo apt install libpcl-dev
```

### Q: Map not displaying?
Check in RViz2:
1. Fixed Frame is set to `map`
2. PointCloud2 topic is set to `/mock_map`
3. Adjust Decay Time or increase Point Size

### Q: How to save the generated map?
You can use pcl_ros's pointcloud_to_pcd node:
```bash
ros2 run pcl_ros pointcloud_to_pcd --ros-args --remap input:=/mock_map
```

## License

GPLv3 - See [LICENSE](LICENSE) file for details

## Author

GitHub@Atticlmr

## Acknowledgments

Based on the original ROS1 version of the mockamap package.

---

[中文文档](README-CN.md)
