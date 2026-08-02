# 2026 川山甲视觉组正式队员考核

方向一：三维视觉与增强现实（ROS 2 + AprilTag AR）。

本项目在 Ubuntu 22.04 与 ROS 2 Humble 下，使用 USB 单目摄像头完成
AprilTag 36h11 检测、相机标定、六维位姿估计、Pose/TF 发布、RViz2
三维可视化，以及具有固定三维偏移的悬浮立方体 AR 投影。

## 完成内容

- AprilTag ID、P0–P3 四角点、中心点与三维坐标轴绘制
- 基于标定内参和 `SOLVEPNP_IPPE_SQUARE` 的六维位姿估计
- 发布 `/apriltag/pose`、`/tf` 与 `/apriltag/markers`
- RViz2 中显示相机模型、标签模型、TF 坐标系和 MarkerArray
- 在 AprilTag 坐标系中绘制固定偏移的 AR 立方体
- 标准 ROS 图像话题模式与低延迟摄像头直连模式
- 固定曝光、MJPG、低缓冲与位姿滤波优化
- 15 cm、18 cm、20 cm 测距误差验证

## 最终环境与参数

| 项目 | 最终配置 |
| --- | --- |
| 操作系统 | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| Python | Python 3.10 |
| 图像库 | OpenCV（含 `aruco`）、NumPy |
| 摄像头 | `/dev/video0`，V4L2 + MJPG |
| 图像尺寸 | 640×480，目标 30 FPS |
| 标签 | AprilTag 36h11，ID 0 |
| 标签边长 | 0.0575 m（外围正方形） |
| 相机内参 | `fx=675.85100`、`fy=683.09268`、`cx=325.73900`、`cy=214.65946` |
| 畸变系数 | `[0.181898, -0.521402, 0.005301, 0.005508, 0]` |

## 仓库结构

```text
.
├── README.md
├── docs/
│   └── DEVELOPMENT_LOG.md
└── src/
    └── apriltag_ar/
        ├── apriltag_ar/
        │   ├── apriltag_detector.py
        │   ├── apriltag_direct.py
        │   └── camera_publisher.py
        ├── config/
        ├── launch/
        ├── package.xml
        └── setup.py
```

## 构建

```bash
git clone https://github.com/sail-liner/2026-CSJ-Vision-Assessment.git
cd 2026-CSJ-Vision-Assessment

source /opt/ros/humble/setup.bash
colcon build --packages-select apriltag_ar
source install/setup.bash
```

## 推荐运行方式

最终演示采用摄像头直连模式，避免 ROS 图像链路排队造成的延迟：

```bash
ros2 launch apriltag_ar apriltag_direct.launch.py
```

也可以运行标准 ROS 图像话题模式：

```bash
ros2 launch apriltag_ar apriltag_demo.launch.py
```

查看处理后的画面：

```bash
rqt_image_view /apriltag/image
```

打开 RViz2：

```bash
rviz2 -d install/apriltag_ar/share/apriltag_ar/config/default.rviz
```

## ROS 输出

- `/apriltag/image`：带检测与 AR 标注的图像
- `/apriltag/pose`：`geometry_msgs/PoseStamped`
- `/apriltag/markers`：`visualization_msgs/MarkerArray`
- `/tf`：`camera_frame → apriltag_<id>`

## 实验结果

| 实际距离 | 测量距离 | 绝对误差 | 相对误差 |
| ---: | ---: | ---: | ---: |
| 15.00 cm | 15.05 cm | 0.05 cm | 0.36% |
| 18.00 cm | 18.63 cm | 0.63 cm | 3.51% |
| 20.00 cm | 20.30 cm | 0.30 cm | 1.51% |

约 10 cm 时因摄像头最小对焦距离限制无法稳定识别，保留为失败案例。实际演示时，
标签外围需要保留白色静区；纯黑背景会降低正面识别稳定性。

## 开发记录

关键调试过程、技术路线变化与最终验证结果见
[`docs/DEVELOPMENT_LOG.md`](docs/DEVELOPMENT_LOG.md)。
