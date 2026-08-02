import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image


class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')

        self.sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.declare_parameter('camera_id', 0)
        self.declare_parameter('frame_id', 'camera_frame')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)

        camera_id = int(self.get_parameter('camera_id').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        fps = float(self.get_parameter('fps').value)

        self.publisher = self.create_publisher(
            Image,
            'camera/image_raw',
            self.sensor_qos
        )

        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            raise RuntimeError(f'无法打开摄像头：{camera_id}')

        self.cap.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*'MJPG')
        )
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_width = int(
            self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        )
        actual_height = int(
            self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        )
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

        # 内部性能统计
        self.stat_start = time.perf_counter()
        self.stat_count = 0
        self.max_read_time = 0.0
        self.max_convert_time = 0.0
        self.max_publish_time = 0.0

        self.timer = self.create_timer(
            1.0 / fps,
            self.publish_frame
        )

        self.get_logger().info(
            '摄像头已启动，正在发布：/camera/image_raw'
        )
        self.get_logger().info(
            f'实际参数：{actual_width}x{actual_height}，'
            f'{actual_fps:.2f} FPS，MJPG，V4L2'
        )

    def publish_frame(self):
        start_read = time.perf_counter()

        success, frame = self.cap.read()

        end_read = time.perf_counter()

        if not success:
            self.get_logger().warning('读取摄像头画面失败')
            return

        message = self.bridge.cv2_to_imgmsg(
            frame,
            encoding='bgr8'
        )
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id

        end_convert = time.perf_counter()

        self.publisher.publish(message)

        end_publish = time.perf_counter()

        self.stat_count += 1

        self.max_read_time = max(
            self.max_read_time,
            end_read - start_read
        )
        self.max_convert_time = max(
            self.max_convert_time,
            end_convert - end_read
        )
        self.max_publish_time = max(
            self.max_publish_time,
            end_publish - end_convert
        )

        elapsed = end_publish - self.stat_start

        if elapsed >= 3.0:
            internal_fps = self.stat_count / elapsed

            self.get_logger().info(
                f'内部发布速率：{internal_fps:.2f} FPS | '
                f'read最大：{self.max_read_time * 1000:.1f} ms | '
                f'convert最大：{self.max_convert_time * 1000:.1f} ms | '
                f'publish最大：{self.max_publish_time * 1000:.1f} ms'
            )

            self.stat_start = end_publish
            self.stat_count = 0
            self.max_read_time = 0.0
            self.max_convert_time = 0.0
            self.max_publish_time = 0.0

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = CameraPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
