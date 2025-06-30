import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer

class ColorDepthConeDetector(Node):
    def __init__(self):
        super().__init__('color_depth_cone_detector')
        self.bridge = CvBridge()

        # 用 message_filters 订阅器同步RGB和深度图
        self.rgb_sub = Subscriber(self, Image, '/aurora/rgb/image_raw')
        self.depth_sub = Subscriber(self, Image, '/aurora/depth/image_raw')

        self.ts = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.synced_callback)

        # 发布检测带框RGB图和掩码图
        self.detect_pub = self.create_publisher(Image, '/detected/cones_rgb', 10)
        self.mask_pub = self.create_publisher(Image, '/detected/cones_mask', 10)

        self.depth_min = 0.1  # 深度最小阈值（米）
        self.depth_max = 1.0  # 深度最大阈值（米）

    def synced_callback(self, rgb_msg, depth_msg):
        try:
            # 转换为OpenCV格式
            rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

            if depth_img.dtype == np.uint16:
                depth_img = depth_img.astype(np.float32) / 1000.0

            # HSV空间筛选橙色
            hsv = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2HSV)
            lower_deep_blue = np.array([110, 150, 15])  # H 110°–, S 150–, V 50–
            upper_deep_blue = np.array([130, 255, 255])  # H –130°, S –, V –
            color_mask = cv2.inRange(hsv, lower_deep_blue, upper_deep_blue)

            # 深度范围掩码
            depth_mask = (depth_img > self.depth_min) & (depth_img < self.depth_max)
            depth_mask = depth_mask.astype(np.uint8) * 255

            # 颜色和深度掩码“与”操作
            combined_mask = cv2.bitwise_and(color_mask, depth_mask)

            # 形态学操作消除噪声
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            clean_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
            clean_mask = cv2.morphologyEx(clean_mask, cv2.MORPH_OPEN, kernel, iterations=1)

            # 查找轮廓并绘制检测框
            contours, _ = cv2.findContours(clean_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            result_img = rgb_img.copy()
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 5000:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(result_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(result_img, 'Cone', (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 发布带框RGB图
            detect_msg = self.bridge.cv2_to_imgmsg(result_img, encoding='bgr8')
            detect_msg.header = rgb_msg.header
            self.detect_pub.publish(detect_msg)

            # 发布掩码图
            mask_msg = self.bridge.cv2_to_imgmsg(clean_mask, encoding='mono8')
            mask_msg.header = rgb_msg.header
            self.mask_pub.publish(mask_msg)

        except Exception as e:
            self.get_logger().error(f"Processing error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ColorDepthConeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
