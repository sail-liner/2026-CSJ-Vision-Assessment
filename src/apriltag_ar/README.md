# AprilTag AR（ROS 2 Humble）

本包使用 USB 摄像头完成 AprilTag 36h11 检测、六维位姿估计、Pose/TF 发布、RViz2 三维显示，以及固定三维偏移立方体的 AR 投影。

## 环境

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- OpenCV（含 `aruco` 模块）
- USB 摄像头：V4L2 + MJPG，640×480，目标 30 FPS

## 构建

```bash
cd ~/apriltag_ar_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select apriltag_ar
source install/setup.bash
```

## 推荐运行方式

直连模式减少 ROS 图像传输造成的排队延迟：

```bash
ros2 run apriltag_ar apriltag_direct
```

也可使用标准相机话题路线：

```bash
ros2 launch apriltag_ar apriltag_demo.launch.py
```

## 输出

- `/apriltag/image`：处理后图像（仅在存在订阅者时发布）
- `/apriltag/pose`：`geometry_msgs/PoseStamped`
- `/apriltag/markers`：`visualization_msgs/MarkerArray`
- `/tf`：`camera_frame` 到 `apriltag_<id>` 的动态变换

## 关键参数

- 标签字典：`DICT_APRILTAG_36h11`
- 标签外围正方形边长：`0.0575 m`
- 图像尺寸：`640×480`
- 相机内参：`fx=675.85100`、`fy=683.09268`、`cx=325.73900`、`cy=214.65946`
- 畸变系数：`[0.181898, -0.521402, 0.005301, 0.005508, 0]`
- 平移滤波系数：`0.90`

## RViz2

启动 RViz2 后，将 `Fixed Frame` 设为 `camera_frame`，添加 `TF`，并订阅 `/apriltag/markers` 的 `MarkerArray`。仓库中的 `config/default.rviz` 会随包安装。

## 项目仓库

完整源码与更新记录见：
[sail-liner/2026-CSJ-Vision-Assessment](https://github.com/sail-liner/2026-CSJ-Vision-Assessment)
