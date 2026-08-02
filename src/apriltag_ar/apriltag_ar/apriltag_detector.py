import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


def rotation_matrix_to_quaternion(matrix):
    trace = np.trace(matrix)

    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (matrix[2, 1] - matrix[1, 2]) / scale
        qy = (matrix[0, 2] - matrix[2, 0]) / scale
        qz = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(
            1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]
        ) * 2.0
        qw = (matrix[2, 1] - matrix[1, 2]) / scale
        qx = 0.25 * scale
        qy = (matrix[0, 1] + matrix[1, 0]) / scale
        qz = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(
            1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]
        ) * 2.0
        qw = (matrix[0, 2] - matrix[2, 0]) / scale
        qx = (matrix[0, 1] + matrix[1, 0]) / scale
        qy = 0.25 * scale
        qz = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(
            1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]
        ) * 2.0
        qw = (matrix[1, 0] - matrix[0, 1]) / scale
        qx = (matrix[0, 2] + matrix[2, 0]) / scale
        qy = (matrix[1, 2] + matrix[2, 1]) / scale
        qz = 0.25 * scale

    return qx, qy, qz, qw


class AprilTagDetector(Node):

    def __init__(self):
        super().__init__('apriltag_detector')

        self.declare_parameter('tag_size', 0.0575)
        self.declare_parameter('fx', 675.85100)
        self.declare_parameter('fy', 683.09268)
        self.declare_parameter('cx', 325.73900)
        self.declare_parameter('cy', 214.65946)

        self.tag_size = float(self.get_parameter('tag_size').value)

        fx = float(self.get_parameter('fx').value)
        fy = float(self.get_parameter('fy').value)
        cx = float(self.get_parameter('cx').value)
        cy = float(self.get_parameter('cy').value)

        self.camera_matrix = np.array([
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        self.distortion = np.array([
            0.181898,
           -0.521402,
            0.005301,
            0.005508,
            0.000000
        ], dtype=np.float64).reshape(5, 1)

        half = self.tag_size / 2.0
        self.object_points = np.array([
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0]
        ], dtype=np.float32)

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        # 位姿指数平滑滤波：数值越小越稳定，但跟随速度越慢
        self.pose_filter_alpha = 0.90
        self.filtered_poses = {}

        self.sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            self.sensor_qos
        )

        self.image_publisher = self.create_publisher(
            Image,
            '/apriltag/image',
            self.sensor_qos
        )

        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/apriltag/pose',
            10
        )

        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/apriltag/markers',
            10
        )

        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_APRILTAG_36h11
        )

        if hasattr(cv2.aruco, 'DetectorParameters'):
            parameters = cv2.aruco.DetectorParameters()
        else:
            parameters = cv2.aruco.DetectorParameters_create()

        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(
                self.dictionary,
                parameters
            )
            self.parameters = None
        else:
            self.detector = None
            self.parameters = parameters

        self.get_logger().info('AprilTag 位姿检测节点已启动')
        self.get_logger().info('位姿话题：/apriltag/pose')
        self.get_logger().info('RViz 模型话题：/apriltag/markers')

    def publish_rviz_markers(self, stamp, detected_marker_ids):
        marker_array = MarkerArray()

        # 相机机身模型，固定在 camera 坐标系
        camera_body = Marker()
        camera_body.header.frame_id = 'camera_frame'
        camera_body.header.stamp = stamp
        camera_body.ns = 'camera_model'
        camera_body.id = 0
        camera_body.type = Marker.CUBE
        camera_body.action = Marker.ADD

        camera_body.pose.position.x = 0.0
        camera_body.pose.position.y = 0.0
        camera_body.pose.position.z = -0.025
        camera_body.pose.orientation.w = 1.0

        camera_body.scale.x = 0.080
        camera_body.scale.y = 0.055
        camera_body.scale.z = 0.040

        camera_body.color.r = 0.10
        camera_body.color.g = 0.40
        camera_body.color.b = 1.00
        camera_body.color.a = 0.90

        marker_array.markers.append(camera_body)

        # 相机镜头模型，镜头朝 camera 坐标系的正 Z 方向
        camera_lens = Marker()
        camera_lens.header.frame_id = 'camera_frame'
        camera_lens.header.stamp = stamp
        camera_lens.ns = 'camera_model'
        camera_lens.id = 1
        camera_lens.type = Marker.CYLINDER
        camera_lens.action = Marker.ADD

        camera_lens.pose.position.x = 0.0
        camera_lens.pose.position.y = 0.0
        camera_lens.pose.position.z = 0.010
        camera_lens.pose.orientation.w = 1.0

        camera_lens.scale.x = 0.035
        camera_lens.scale.y = 0.035
        camera_lens.scale.z = 0.035

        camera_lens.color.r = 0.00
        camera_lens.color.g = 0.85
        camera_lens.color.b = 1.00
        camera_lens.color.a = 1.00

        marker_array.markers.append(camera_lens)

        # 为每个检测成功的 AprilTag 创建平面模型
        for marker_id in detected_marker_ids:
            frame_id = f'apriltag_{marker_id}'
            namespace = f'apriltag_{marker_id}_model'

            # 白色标签底板
            tag_plate = Marker()
            tag_plate.header.frame_id = frame_id
            tag_plate.header.stamp = stamp
            tag_plate.ns = namespace
            tag_plate.id = 0
            tag_plate.type = Marker.CUBE
            tag_plate.action = Marker.ADD

            tag_plate.pose.position.x = 0.0
            tag_plate.pose.position.y = 0.0
            tag_plate.pose.position.z = 0.0
            tag_plate.pose.orientation.w = 1.0

            tag_plate.scale.x = self.tag_size
            tag_plate.scale.y = self.tag_size
            tag_plate.scale.z = 0.003

            tag_plate.color.r = 1.0
            tag_plate.color.g = 1.0
            tag_plate.color.b = 1.0
            tag_plate.color.a = 1.0

            # 检测消失后，模型在短时间内自动移除
            tag_plate.lifetime.sec = 1
            tag_plate.lifetime.nanosec = 0

            marker_array.markers.append(tag_plate)

            # 黑色中心区域，使模型更像 AprilTag
            tag_center = Marker()
            tag_center.header.frame_id = frame_id
            tag_center.header.stamp = stamp
            tag_center.ns = namespace
            tag_center.id = 1
            tag_center.type = Marker.CUBE
            tag_center.action = Marker.ADD

            tag_center.pose.position.x = 0.0
            tag_center.pose.position.y = 0.0
            tag_center.pose.position.z = 0.002
            tag_center.pose.orientation.w = 1.0

            tag_center.scale.x = self.tag_size * 0.70
            tag_center.scale.y = self.tag_size * 0.70
            tag_center.scale.z = 0.003

            tag_center.color.r = 0.03
            tag_center.color.g = 0.03
            tag_center.color.b = 0.03
            tag_center.color.a = 1.0

            tag_center.lifetime.sec = 1
            tag_center.lifetime.nanosec = 0

            marker_array.markers.append(tag_center)

            # 标签 ID 文字
            tag_text = Marker()
            tag_text.header.frame_id = frame_id
            tag_text.header.stamp = stamp
            tag_text.ns = namespace
            tag_text.id = 2
            tag_text.type = Marker.TEXT_VIEW_FACING
            tag_text.action = Marker.ADD

            tag_text.pose.position.x = 0.0
            tag_text.pose.position.y = -self.tag_size * 0.80
            tag_text.pose.position.z = 0.015
            tag_text.pose.orientation.w = 1.0

            tag_text.scale.z = 0.020
            tag_text.color.r = 1.0
            tag_text.color.g = 0.85
            tag_text.color.b = 0.0
            tag_text.color.a = 1.0
            tag_text.text = f'AprilTag ID {marker_id}'

            tag_text.lifetime.sec = 1
            tag_text.lifetime.nanosec = 0

            marker_array.markers.append(tag_text)

        self.marker_publisher.publish(marker_array)


    def draw_ar_cube(self, frame, rvec, tvec):
        """在 AprilTag 坐标系中绘制一个具有固定三维偏移的悬浮立方体。"""

        # 立方体边长
        side = self.tag_size * 0.55
        half_side = side / 2.0

        # 立方体中心相对 AprilTag 中心的固定偏移
        # x：位于标签右侧
        # y：与标签中心同高
        # z：悬浮在标签平面上方
        center_x = self.tag_size * 0.90
        center_y = 0.0
        bottom_z = self.tag_size * 0.18
        top_z = bottom_z + side

        # 立方体八个顶点，全部定义在 AprilTag 坐标系中
        cube_points_3d = np.array([
            [center_x - half_side, center_y + half_side, bottom_z],
            [center_x + half_side, center_y + half_side, bottom_z],
            [center_x + half_side, center_y - half_side, bottom_z],
            [center_x - half_side, center_y - half_side, bottom_z],

            [center_x - half_side, center_y + half_side, top_z],
            [center_x + half_side, center_y + half_side, top_z],
            [center_x + half_side, center_y - half_side, top_z],
            [center_x - half_side, center_y - half_side, top_z],
        ], dtype=np.float32)

        # 根据当前 AprilTag 位姿，把三维顶点投影到图像平面
        projected_points, _ = cv2.projectPoints(
            cube_points_3d,
            rvec,
            tvec,
            self.camera_matrix,
            self.distortion
        )

        points = np.round(
            projected_points.reshape(-1, 2)
        ).astype(np.int32)

        # 半透明填充立方体顶面
        overlay = frame.copy()
        cv2.fillConvexPoly(
            overlay,
            points[[4, 5, 6, 7]],
            (190, 40, 190),
            cv2.LINE_AA
        )
        cv2.addWeighted(
            overlay,
            0.28,
            frame,
            0.72,
            0.0,
            frame
        )

        bottom_edges = [
            (0, 1), (1, 2), (2, 3), (3, 0)
        ]
        top_edges = [
            (4, 5), (5, 6), (6, 7), (7, 4)
        ]
        vertical_edges = [
            (0, 4), (1, 5), (2, 6), (3, 7)
        ]

        # 底面：橙色
        for start, end in bottom_edges:
            cv2.line(
                frame,
                tuple(points[start]),
                tuple(points[end]),
                (0, 180, 255),
                2,
                cv2.LINE_AA
            )

        # 顶面：紫色
        for start, end in top_edges:
            cv2.line(
                frame,
                tuple(points[start]),
                tuple(points[end]),
                (255, 80, 255),
                3,
                cv2.LINE_AA
            )

        # 四根竖边：青色
        for start, end in vertical_edges:
            cv2.line(
                frame,
                tuple(points[start]),
                tuple(points[end]),
                (255, 255, 0),
                2,
                cv2.LINE_AA
            )

        # 标注虚拟物体
        label_x = int(np.mean(points[:, 0])) - 32
        label_y = int(np.min(points[:, 1])) - 10

        label_x = max(
            5,
            min(label_x, frame.shape[1] - 80)
        )
        label_y = max(20, label_y)

        cv2.putText(
            frame,
            'AR CUBE',
            (label_x, label_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (0, 0, 0),
            3,
            cv2.LINE_AA
        )

        cv2.putText(
            frame,
            'AR CUBE',
            (label_x, label_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 255, 255),
            1,
            cv2.LINE_AA
        )

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray,
                self.dictionary,
                parameters=self.parameters
            )

        detected_marker_ids = []

        if ids is not None:
            for marker_corners, marker_id in zip(corners, ids.flatten()):
                image_points = marker_corners.reshape(4, 2).astype(
                    np.float32
                )

                pixel_points = image_points.astype(int)

                # 绘制 AprilTag 外轮廓
                cv2.polylines(
                    frame,
                    [pixel_points.reshape((-1, 1, 2))],
                    True,
                    (0, 220, 0),
                    2,
                    cv2.LINE_AA
                )

                # 四个角点标签的位置偏移
                label_offsets = [
                    (-34, -12),   # P0：左上
                    (10, -12),    # P1：右上
                    (10, 24),     # P2：右下
                    (-34, 24),    # P3：左下
                ]

                for index, point in enumerate(pixel_points):
                    x, y = int(point[0]), int(point[1])
                    label_x = x + label_offsets[index][0]
                    label_y = y + label_offsets[index][1]

                    # 角点圆
                    cv2.circle(
                        frame,
                        (x, y),
                        5,
                        (0, 255, 255),
                        -1,
                        cv2.LINE_AA
                    )

                    # 黑色描边，提高文字可读性
                    cv2.putText(
                        frame,
                        f'P{index}',
                        (label_x, label_y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.48,
                        (0, 0, 0),
                        3,
                        cv2.LINE_AA
                    )

                    cv2.putText(
                        frame,
                        f'P{index}',
                        (label_x, label_y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.48,
                        (0, 255, 255),
                        1,
                        cv2.LINE_AA
                    )

                # 计算标签中心
                center = np.mean(pixel_points, axis=0).astype(int)
                center_x = int(center[0])
                center_y = int(center[1])

                # 在标签上方显示 ID，并添加背景框
                id_text = f'ID {marker_id}'
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.58
                thickness = 2

                text_size, baseline = cv2.getTextSize(
                    id_text,
                    font,
                    font_scale,
                    thickness
                )

                text_width, text_height = text_size
                top_y = int(np.min(pixel_points[:, 1]))
                id_x = center_x - text_width // 2
                id_y = max(text_height + 12, top_y - 14)

                cv2.rectangle(
                    frame,
                    (id_x - 7, id_y - text_height - 7),
                    (id_x + text_width + 7, id_y + baseline + 5),
                    (25, 25, 25),
                    -1
                )

                cv2.putText(
                    frame,
                    id_text,
                    (id_x, id_y),
                    font,
                    font_scale,
                    (255, 255, 255),
                    thickness,
                    cv2.LINE_AA
                )

                success, rvec, tvec = cv2.solvePnP(
                    self.object_points,
                    image_points,
                    self.camera_matrix,
                    self.distortion,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )

                if not success:
                    self.get_logger().warning(
                        f'AprilTag {marker_id}: solvePnP 失败'
                    )
                    continue

                # 对同一 AprilTag 的旋转和平移进行指数平滑
                marker_key = int(marker_id)
                alpha = self.pose_filter_alpha

                if marker_key in self.filtered_poses:
                    _, previous_tvec = self.filtered_poses[marker_key]

                    # 平移向量可以直接进行指数平滑
                    tvec = (
                        alpha * tvec
                        + (1.0 - alpha) * previous_tvec
                    )

                # 旋转向量不能直接线性平均，否则可能跨越正负 pi
                # 导致 RViz 中模型连续翻转或旋转多圈
                self.filtered_poses[marker_key] = (
                    rvec.copy(),
                    tvec.copy()
                )

                self.get_logger().info(
                    f'AprilTag {marker_id}: 位姿成功，'
                    f'x={float(tvec[0, 0]):.3f}, '
                    f'y={float(tvec[1, 0]):.3f}, '
                    f'z={float(tvec[2, 0]):.3f}',
                    throttle_duration_sec=1.0
                )

                cv2.drawFrameAxes(
                    frame,
                    self.camera_matrix,
                    self.distortion,
                    rvec,
                    tvec,
                    self.tag_size * 0.38
                )

                # 绘制固定在 AprilTag 世界坐标系中的虚拟立方体
                self.draw_ar_cube(frame, rvec, tvec)

                # 在坐标轴上层重新绘制中心点
                cv2.circle(
                    frame,
                    (center_x, center_y),
                    8,
                    (255, 255, 255),
                    -1,
                    cv2.LINE_AA
                )

                cv2.circle(
                    frame,
                    (center_x, center_y),
                    4,
                    (255, 0, 255),
                    -1,
                    cv2.LINE_AA
                )

                center_text = 'CENTER'
                center_font = cv2.FONT_HERSHEY_SIMPLEX
                center_scale = 0.42
                center_thickness = 1

                center_size, center_baseline = cv2.getTextSize(
                    center_text,
                    center_font,
                    center_scale,
                    center_thickness
                )

                center_text_width, center_text_height = center_size

                center_label_x = min(
                    center_x + 28,
                    frame.shape[1] - center_text_width - 12
                )

                center_label_y = max(
                    center_y - 22,
                    center_text_height + 10
                )

                cv2.line(
                    frame,
                    (center_x + 5, center_y - 5),
                    (center_label_x - 5, center_label_y),
                    (255, 0, 255),
                    2,
                    cv2.LINE_AA
                )

                cv2.rectangle(
                    frame,
                    (
                        center_label_x - 5,
                        center_label_y - center_text_height - 6
                    ),
                    (
                        center_label_x + center_text_width + 5,
                        center_label_y + center_baseline + 4
                    ),
                    (25, 25, 25),
                    -1
                )

                cv2.putText(
                    frame,
                    center_text,
                    (center_label_x, center_label_y),
                    center_font,
                    center_scale,
                    (255, 255, 255),
                    center_thickness,
                    cv2.LINE_AA
                )

                rotation_matrix, _ = cv2.Rodrigues(rvec)
                qx, qy, qz, qw = rotation_matrix_to_quaternion(
                    rotation_matrix
                )

                pose = PoseStamped()
                pose.header = msg.header
                pose.pose.position.x = float(tvec[0, 0])
                pose.pose.position.y = float(tvec[1, 0])
                pose.pose.position.z = float(tvec[2, 0])
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                self.pose_publisher.publish(pose)

                transform = TransformStamped()
                transform.header = msg.header
                transform.child_frame_id = f'apriltag_{marker_id}'
                transform.transform.translation.x = float(tvec[0, 0])
                transform.transform.translation.y = float(tvec[1, 0])
                transform.transform.translation.z = float(tvec[2, 0])
                transform.transform.rotation.x = qx
                transform.transform.rotation.y = qy
                transform.transform.rotation.z = qz
                transform.transform.rotation.w = qw
                self.tf_broadcaster.sendTransform(transform)
                detected_marker_ids.append(int(marker_id))

        self.publish_rviz_markers(
            msg.header.stamp,
            detected_marker_ids
        )

        # 直接在检测进程中显示，不经过 ROS 图像传输
        cv2.imshow('AprilTag AR', frame)
        cv2.waitKey(1)

        # 只有存在订阅者时才进行原始图像转换和发布
        # 本地 OpenCV 窗口不需要经过 ROS 图像传输
        if self.image_publisher.get_subscription_count() > 0:
            output = self.bridge.cv2_to_imgmsg(
                frame,
                encoding='bgr8'
            )
            output.header = msg.header
            self.image_publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
