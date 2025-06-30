import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class ConeAvoidanceNode(Node):
    def __init__(self):
        super().__init__('cone_avoidance_node')
        self.bridge = CvBridge()

        # 订阅深度图
        self.sub = self.create_subscription(
            Image,
            '/aurora/depth/image_raw',
            self.depth_callback,
            10
        )

        # 发布伪彩色图和检测图
        self.color_pub   = self.create_publisher(Image, '/depth/image_colormap', 10)
        self.detect_pub  = self.create_publisher(Image, '/depth/cone_detected', 10)
        # 发布运动控制命令
        self.cmd_pub     = self.create_publisher(Twist, '/cmd_vel', 10)

        self.max_depth   = 0.5   # 可视化最大深度（米）
        self.visual_debug= False # 是否打开 OpenCV 窗口

        #避障参数
        self.forward_speed = 0.2  # 无障碍时前进速度
        self.turn_speed = 0.5  # 避障时转弯角速度
        self.state = 'FORWARD'  # 初始状态
        self.recenter_start_time = None
        self.recenter_duration = 3  # 回正持续时间（秒）
        self.last_turn_direction = 0  # -1左转，1右转，0无

    def depth_callback(self, msg):
        try:
            # ---------------- 深度图预处理 ----------------
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if depth_img.dtype == np.uint16:
                depth_img = depth_img.astype(np.float32) / 1000.0
            depth_img[depth_img > self.max_depth] = np.nan
            depth_img = np.nan_to_num(depth_img, nan=self.max_depth)
            norm = np.clip(depth_img / self.max_depth, 0, 1)
            depth_8u = (norm * 255).astype(np.uint8)
            color_map = cv2.applyColorMap(depth_8u, cv2.COLORMAP_JET)

            # ---------------- 锥形检测 ----------------
            gray = cv2.cvtColor(color_map, cv2.COLOR_BGR2GRAY)
            _, binary = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)

            # 闭 + 开 操作
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)
            binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN,  kernel, iterations=1)

            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            detected_img = color_map.copy()

            img_h, img_w = gray.shape
            obstacle_center_x = None

            # 遍历筛选锥形轮廓，记录第一个障碍中心
            for cnt in contours:
                if cv2.contourArea(cnt) < 800:
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                if h / float(w) > 1.2:
                    # 绘制检测框
                    cv2.drawContours(detected_img, [cnt], -1, (0, 255, 0), 2)
                    cv2.putText(detected_img, "Cone", (x, y-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                    # 记录中心点（仅第一个）
                    M = cv2.moments(cnt)
                    if M['m00'] > 0:
                        obstacle_center_x = int(M['m10']/M['m00'])
                    break

            # ---------------- 发布图像话题 ----------------
            color_msg = self.bridge.cv2_to_imgmsg(color_map, encoding='bgr8')
            color_msg.header = msg.header
            self.color_pub.publish(color_msg)

            detect_msg = self.bridge.cv2_to_imgmsg(detected_img, encoding='bgr8')
            detect_msg.header = msg.header
            self.detect_pub.publish(detect_msg)

            # ---------------- 避障控制 ----------------
            twist = Twist()
            if obstacle_center_x is None:
                if self.state == 'AVOIDING':
                    # 障碍消失，开始回正
                    self.state = 'RECENTERING'
                    self.recenter_start_time = self.get_clock().now()
                if self.state == 'RECENTERING':
                    elapsed = (self.get_clock().now() - self.recenter_start_time).nanoseconds / 1e9
                    if elapsed < self.recenter_duration:
                        twist.linear.x = self.forward_speed
                        twist.angular.z = -self.last_turn_direction * self.turn_speed
                    else:
                        self.state = 'FORWARD'
                        twist.linear.x = self.forward_speed
                        twist.angular.z = 0.0
                else:
                    # 正常前进
                    twist.linear.x = self.forward_speed
                    twist.angular.z = 0.0
            else:
                # 遇到障碍，转向避让
                self.state = 'AVOIDING'
                if obstacle_center_x < img_w // 2:
                    twist.linear.x = self.forward_speed
                    twist.angular.z = -self.turn_speed
                    self.last_turn_direction = -1
                else:
                    twist.linear.x = self.forward_speed
                    twist.angular.z = self.turn_speed
                    self.last_turn_direction = 1
            self.cmd_pub.publish(twist)

            # ---------------- 可视化调试 ----------------
            if self.visual_debug:
                cv2.circle(detected_img, (obstacle_center_x or img_w//2, img_h//2),
                           5, (0,0,255), -1)
                cv2.imshow("ColorMap", color_map)
                cv2.imshow("Detected", detected_img)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Processing error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ConeAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
