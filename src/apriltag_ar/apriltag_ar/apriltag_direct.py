import cv2
import rclpy

from apriltag_ar.apriltag_detector import AprilTagDetector


class AprilTagDirect(AprilTagDetector):
    """直接读取摄像头，不通过 ROS 图像话题传输。"""

    def __init__(self):
        super().__init__()

        # 删除原来的 /camera/image_raw 订阅
        if getattr(self, 'subscription', None) is not None:
            self.destroy_subscription(self.subscription)
            self.subscription = None

        self.declare_parameter('camera_id', 0)
        self.declare_parameter('camera_frame_id', 'camera_frame')
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_fps', 30.0)

        camera_id = int(self.get_parameter('camera_id').value)
        self.camera_frame_id = str(
            self.get_parameter('camera_frame_id').value
        )
        width = int(self.get_parameter('camera_width').value)
        height = int(self.get_parameter('camera_height').value)
        fps = float(self.get_parameter('camera_fps').value)

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

        # 摄像头本身控制帧率，定时器只负责持续读取
        self.capture_timer = self.create_timer(
            1.0 / fps,
            self.capture_frame
        )

        self.get_logger().info(
            '已启用摄像头直连模式，不再传输 /camera/image_raw'
        )
        self.get_logger().info(
            f'摄像头参数：{actual_width}x{actual_height}，'
            f'{actual_fps:.2f} FPS，MJPG'
        )

    def capture_frame(self):
        success, frame = self.cap.read()

        if not success:
            self.get_logger().warning(
                '摄像头读取失败',
                throttle_duration_sec=1.0
            )
            return

        # 构造本地消息，仅在本进程内交给原检测函数
        message = self.bridge.cv2_to_imgmsg(
            frame,
            encoding='bgr8'
        )
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.camera_frame_id

        self.image_callback(message)

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()

        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = AprilTagDirect()
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
