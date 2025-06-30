import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ConeDetector(Node):
    def __init__(self):
        super().__init__('cone_detector_node')
        self.bridge = CvBridge()

        # 订阅深度图
        self.sub = self.create_subscription(
            Image,
            '/aurora/depth/image_raw',
            self.depth_callback,
            10
        )

        # 发布伪彩色图像
        self.color_pub = self.create_publisher(
            Image,
            '/depth/image_colormap',
            10
        )

        # 发布带有检测框的图像
        self.detect_pub = self.create_publisher(
            Image,
            '/depth/cone_detected',
            10
        )

        self.max_depth = 1.0  # 最大深度阈值（米）
        self.visual_debug = False

    def depth_callback(self, msg):
        try:
            # 转换为 OpenCV 格式
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # 若为16位深度图，转为单位米
            if depth_img.dtype == np.uint16:
                depth_img = depth_img.astype(np.float32) / 1000.0

            # 屏蔽超出范围
            depth_img[depth_img > self.max_depth] = np.nan

            # NaN 替换为 max_depth（避免归一化出错）
            depth_img = np.nan_to_num(depth_img, nan=self.max_depth)

            # 归一化并转为伪彩色
            norm = np.clip(depth_img / self.max_depth, 0, 1)
            depth_8u = (norm * 255).astype(np.uint8)
            color_map = cv2.applyColorMap(depth_8u, cv2.COLORMAP_JET)

            # === 图像处理用于锥形检测 ===
            gray = cv2.cvtColor(color_map, cv2.COLOR_BGR2GRAY)

            # 阈值提取较浅区域（靠近相机）
            _, binary = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)

            # 膨胀腐蚀（闭操作 + 开操作）
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            binary_closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)
            binary_cleaned = cv2.morphologyEx(binary_closed, cv2.MORPH_OPEN, kernel, iterations=1)

            # 查找轮廓
            contours, _ = cv2.findContours(binary_cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            result = color_map.copy()

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 800:
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = h / float(w)
                if aspect_ratio > 1.2:
                    cv2.drawContours(result, [cnt], -1, (0, 255, 0), 2)
                    cv2.putText(result, "Cone", (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 发布伪彩色图像
            color_msg = self.bridge.cv2_to_imgmsg(color_map, encoding='bgr8')
            color_msg.header = msg.header
            self.color_pub.publish(color_msg)

            # 发布检测图像
            detect_msg = self.bridge.cv2_to_imgmsg(result, encoding='bgr8')
            detect_msg.header = msg.header
            self.detect_pub.publish(detect_msg)

            self.get_logger().info('Published.')
            if self.visual_debug:
                cv2.imshow("Binary", binary)
                cv2.imshow("Detected", result)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ConeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
