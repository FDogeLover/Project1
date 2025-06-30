import rclpy
import cv2
import numpy as np
import cv_bridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.bridge = cv_bridge.CvBridge()

        # 订阅相机图像
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        # 订阅避障节点发布的速度指令
        self.obstacle_cmd_sub = self.create_subscription(Twist, '/obstacle_avoidance/cmd_vel', self.obstacle_cmd_callback, 10)
        # 发布最终的速度指令
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # 发布调试图像
        self.debug_pub = self.create_publisher(Image, '/processed_image', 10)

        self.last_err = 0.0
        self.obstacle_cmd = None  # 缓存避障指令

        self.get_logger().info("✅ Line follower node started.")

    def preprocess_black_line(self, roi):
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([100, 255, 60])
        mask = cv2.inRange(hsv, lower_black, upper_black)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        return mask

    def obstacle_cmd_callback(self, msg: Twist):
        # 缓存避障速度指令
        self.obstacle_cmd = msg

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape
        roi = frame[int(h * 0.7):, int(w * 0.2):int(w * 0.8)]
        mask = self.preprocess_black_line(roi)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        M = cv2.moments(mask)
        twist = Twist()

        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(roi, (cx, cy), 6, (0, 0, 255), -1)
            roi_w = roi.shape[1]
            err = cx - (roi_w / 2)
            derivative = err - self.last_err
            self.last_err = err

            Kp = 0.005
            Kd = 0.001

            max_speed = 0.5
            min_speed = 0.1
            twist.linear.x = max(min_speed, max_speed - abs(err) / (roi_w / 2) * 0.1)
            twist.angular.z = -(Kp * err + Kd * derivative)
            twist.angular.z = float(np.clip(twist.angular.z, -1.5, 1.5))

            # self.get_logger().info(f"[跟踪] x={twist.linear.x:.2f}, z={twist.angular.z:.2f}")
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            # self.get_logger().warn("⚠️ 未检测到黑线，停止")

        # 优先发布避障指令（非零才覆盖）
        if self.obstacle_cmd is not None:
            # 判断避障cmd是否非零（这里判断角速度是否不为0）
            if abs(self.obstacle_cmd.angular.z) > 1e-4:
                # self.get_logger().info(f"[避障优先] 使用避障cmd_vel: linear.x={self.obstacle_cmd.linear.x:.2f}, angular.z={self.obstacle_cmd.angular.z:.2f}")
                twist = self.obstacle_cmd

        # 发布最终速度指令
        self.cmd_pub.publish(twist)

        # 发布调试图像
        debug_roi = roi.copy()
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(debug_roi, contours, -1, (0, 255, 0), 1)
        debug_image = cv2.resize(debug_roi, (w, int(h * 0.3)))
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8'))

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
