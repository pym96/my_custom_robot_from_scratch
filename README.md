# my_cutom_robot - ROS2 移动机器人仿真项目

这是一个基于ROS2的移动机器人仿真项目，包含了URDF模型定义、Gazebo仿真环境、传感器配置以及机器人控制功能。

## 项目简介

本项目实现了一个带有轮式底盘的移动机器人，配备了以下功能和传感器：
- 差速驱动底盘（两个驱动轮和一个万向轮）
- 激光雷达（LIDAR）传感器，用于环境感知和导航
- 相机传感器，用于视觉检测

项目支持在Gazebo仿真环境中运行，同时提供RViz2可视化和控制界面。

## 目录结构

- `config/`: 项目配置文件
- `description/`: 机器人模型描述文件（URDF/XACRO）
  - `robot_core.xacro`: 机器人核心结构定义
  - `camera.xacro`: 相机传感器定义
  - `lidar.xacro`: 激光雷达传感器定义
  - `robot.urdf.xacro`: 主要的机器人描述文件
  - `gazebo_control.xacro`: Gazebo控制插件配置
  - `inertial_macros.xacro`: 惯性参数宏定义
- `launch/`: 启动文件
  - `rsp.launch.py`: 机器人状态发布器启动文件
  - `launch_sim.launch.py`: 完整仿真环境启动文件
- `worlds/`: Gazebo仿真世界
  - `first_world.world`: 基本仿真世界
  - `world_with_lidar`: 配置有激光雷达的仿真世界

## 系统要求

- ROS2 (Humble 或兼容版本)
- Gazebo
- 相关ROS2包：
  - gazebo_ros
  - robot_state_publisher
  - joint_state_publisher_gui
  - teleop_twist_keyboard (用于键盘控制)
  - rviz2
  - image_transport_plugins (用于图像传输)
  - rqt_image_view (用于图像可视化)

## 使用方法

### 启动完整仿真

一键启动完整仿真环境：

```bash
ros2 launch my_cutom_robot launch_sim.launch.py
```

### 分步启动各组件

1. 启动Gazebo仿真环境：

```bash
ros2 launch gazebo_ros gazebo.launch.py
```

2. 启动机器人状态发布器（开启仿真时间同步）：

```bash
ros2 launch my_cutom_robot rsp.launch.py use_sim_time:=true
```

3. 将机器人模型加载到Gazebo中：

```bash
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity demo
```

4. 查看机器人关节状态（可选）：

```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

5. 键盘控制机器人移动：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 图像查看与处理

安装图像处理插件并查看机器人相机图像：

```bash
sudo apt install ros-humble-image-transport-plugins
sudo apt install ros-humble-rqt-image-view
ros2 run rqt_image_view rqt_image_view
```

## 注意事项

- 确保使用`use_sim_time:=true`参数以同步RViz2、Gazebo和其他组件的时间戳
- 机器人使用差速驱动控制，可通过`teleop_twist_keyboard`节点进行控制
- 相机和激光雷达数据可在相应的ROS2话题中查看

## Tips:

在仿真中使joint可见，运行以下命令：

    ros2 run joint_state_publisher_gui joint_state_publisher_gui 

let your joint visible.

运行Gazebo仿真

    ros2 launch gazebo_ros gazebo.launch.py

Open your gazebo

打开sim_time:=true, 使得rivz2 和 Gazobo 乃至未来的 foxglove在时间戳上进行话题信息上的同步，以便仿真使用 

    ros2 launch my_cutom_robot rsp.launch.py use_sim_time:=true

Sync your rviz2 with Gazebo 

将话题 robot_description 中的robot model "放"到Gazobo仿真中， 举一个很简单的例子，使在 rviz2 那些带颜色的 urdf在 gazebo中也能带颜色（这个比较肤浅，当然还有使机器人在仿真中更真实的物理特性，比如静态摩擦力参数等）

    ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity demo
Spawn your urdf robot into gazebo 


    ros2 run teleop_twist_keyboard teleop_twist_keyboard 
Move your robot in Gazebo


    sudo apt install ros-humble-image-transport-plugis
    sudo apt install ros-humble-rqt-image-view
    ros2 run rqt_image_view rqt_image_view 

To compress your image when transport them in different topics

## Due to the complexity of publishing different nodes once
Use:
    launch_sim.launch.py
