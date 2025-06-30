import cv2
import numpy as np

# 读取图像
img = cv2.imread('1.png')
if img is None:
    raise ValueError("图像读取失败！请检查文件路径。")

# 缩小图像方便调试
img_resized = cv2.resize(img, (640, 480))

# 转灰度图（便于阈值化）
gray = cv2.cvtColor(img_resized, cv2.COLOR_BGR2GRAY)

# 阈值提取近距离（浅色区域）
_, binary = cv2.threshold(gray, 140, 255, cv2.THRESH_BINARY)

# === 添加腐蚀 + 膨胀操作 ===
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

# 步骤1：闭操作填补锥体区域内部空洞
binary_closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)

# 步骤2：开操作去除小噪声点（可选）
binary_cleaned = cv2.morphologyEx(binary_closed, cv2.MORPH_OPEN, kernel, iterations=1)

# 查找轮廓
contours, _ = cv2.findContours(binary_cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# 绘制检测结果
result = img_resized.copy()

for cnt in contours:
    area = cv2.contourArea(cnt)
    if area < 5000:
        continue  # 忽略太小区域

    x, y, w, h = cv2.boundingRect(cnt)
    aspect_ratio = h / float(w)

    if aspect_ratio > 1.2:  # 偏长锥形
        cv2.drawContours(result, [cnt], -1, (0, 255, 0), 2)
        cv2.putText(result, "Cone", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

# 显示各阶段图像
cv2.imshow("Original", img_resized)
cv2.imshow("Gray Binary", binary)
cv2.imshow("Morph Closed", binary_closed)
cv2.imshow("Cleaned Binary", binary_cleaned)
cv2.imshow("Detected Cones", result)
cv2.waitKey(0)
cv2.destroyAllWindows()
