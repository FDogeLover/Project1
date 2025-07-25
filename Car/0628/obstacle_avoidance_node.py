import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber
import time


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.bridge = CvBridge()

        self.rgb_sub = Subscriber(self, Image, '/aurora/rgb/image_raw')
        self.depth_sub = Subscriber(self, Image, '/aurora/depth/image_raw')
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.image_callback)

        self.cmd_pub = self.create_publisher(Twist, '/obstacle_avoidance/cmd_vel', 10)

        self.lower_blue = np.array([110, 150, 15])
        self.upper_blue = np.array([130, 255, 255])

        self.depth_min = 0.1
        self.depth_max = 1.0

        self.forward_speed = 1.0
        self.turn_speed = 0.5

        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))

        # 新增：记录最近转向，None/ 'left' / 'right'
        self.last_turn_direction = None

        # 新增：回正控制，记录回正开始时间
        self.returning_start_time = None
        self.returning_duration = 0.5  # 回正动作持续0.5秒

        self.max_delta = 0  # 最大偏移量，用于动态回正时间计算
        self.base_return_time = 0.3  # 基础回正时间
        self.k = 0.5  # 调整系数，回正时间 ∝ 避障强度

    def image_callback(self, rgb_msg, depth_msg):
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) / 1000.0

            small_rgb = cv2.resize(rgb, (320, 240))
            small_depth = cv2.resize(depth, (320, 240))

            hsv = cv2.cvtColor(small_rgb, cv2.COLOR_BGR2HSV)
            color_mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)

            depth_mask = ((small_depth > self.depth_min) &
                          (small_depth < self.depth_max)).astype(np.uint8) * 255

            mask = cv2.bitwise_and(color_mask, depth_mask)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=1)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cx = None
            for cnt in contours:
                if cv2.contourArea(cnt) < 500:
                    continue
                M = cv2.moments(cnt)
                if M['m00'] == 0:
                    continue
                cx = int(M['m10'] / M['m00'])
                break

            twist = Twist()
            center_x = small_rgb.shape[1] // 2

            current_time = time.time()

            if cx is not None:
                delta = cx - center_x
                abs_norm = min(abs(delta) / center_x, 1.0)  # 限制在0~1之间

                # 越靠中心 → abs_norm 越小 → 角速度越大
                angular_z = self.turn_speed * (1 - abs_norm)

                # 转向方向
                if delta < 0:
                    angular_z = -angular_z  # 障碍偏左，右转

                # 可选：线速度越危险越慢（可保留或设定恒定值）
                linear_x = self.forward_speed * abs_norm  # 越边缘越快
                linear_x = max(linear_x, 0.05)  # 最小线速度保护

                twist.linear.x = linear_x
                twist.angular.z = angular_z

                # 记录方向
                self.last_turn_direction = 'right' if delta < 0 else 'left'
                self.returning_start_time = None
                # 在避障过程中实时更新
                self.max_delta = max(self.max_delta, abs(cx - center_x))

            else:
                # 无障碍，判断是否需要回正
                if self.last_turn_direction is not None:
                    self.returning_duration = self.base_return_time + self.k * (self.max_delta / center_x)
                    # 开始或继续回正
                    if self.returning_start_time is None:
                        self.max_delta = 0  # 开启新的避障轮次
                        self.returning_start_time = current_time

                    elapsed = current_time - self.returning_start_time
                    if elapsed < self.returning_duration:
                        # 反方向旋转回正
                        twist.linear.x = 0.5 * self.forward_speed
                        if self.last_turn_direction == 'left':
                            twist.angular.z = -self.turn_speed  # 反向右转
                        else:
                            twist.angular.z = self.turn_speed  # 反向左转
                    else:
                        # 回正完成，直行并重置状态
                        twist.linear.x = self.forward_speed
                        twist.angular.z = 0.0
                        self.last_turn_direction = None
                        self.returning_start_time = None
                else:
                    # 无需回正，直行
                    twist.linear.x = self.forward_speed
                    twist.angular.z = 0.0

            self.cmd_pub.publish(twist)

        except Exception as e:
            self.get_logger().error(f"Error in callback: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
