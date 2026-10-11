#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class TrafficLightDetection(Node):

    def __init__(self):
        super().__init__('traffic_light_detection')

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/camera/front_camera/image_raw',
            self.image_callback,
            10
        )

        self.state_pub = self.create_publisher(
            String,
            '/traffic_light/detected_state',
            10
        )

        # 连续多帧确认，防止动画切换瞬间误判
        self.candidate_state = 'UNKNOWN'
        self.candidate_count = 0
        self.stable_state = 'UNKNOWN'

        self.get_logger().info(
            '全新红绿灯视觉识别节点已启动：左=RED，中=YELLOW，右=GREEN'
        )

    def image_callback(self, msg):

        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().error(
                '摄像头图像转换失败: {}'.format(e)
            )
            return

        state, scores = self.detect_light(frame)

        # 临时诊断：每约 1 秒输出一次检测结果
        if not hasattr(self, 'debug_counter'):
            self.debug_counter = 0

        self.debug_counter += 1

        if self.debug_counter >= 8:
            self.debug_counter = 0
            self.get_logger().info(
                f'检测诊断 | RED={scores[0]:.0f} '
                f'YELLOW={scores[1]:.0f} '
                f'GREEN={scores[2]:.0f} '
                f'判断={state}'
            )

        # =========================
        # 连续 3 帧确认
        # =========================
        if state == self.candidate_state:
            self.candidate_count += 1
        else:
            self.candidate_state = state
            self.candidate_count = 1

        if self.candidate_count < 3:
            return

        if state == 'UNKNOWN':
            return

        # 每一帧都发布稳定状态，导航程序以后可以直接读取
        state_msg = String()
        state_msg.data = state
        self.state_pub.publish(state_msg)

        # 只有状态变化时打印，避免终端疯狂刷屏
        if state != self.stable_state:

            self.stable_state = state

            self.get_logger().info(
                '\n'
                '============================\n'
                '红绿灯视觉识别成功\n'
                '状态: {}\n'
                '左灯 RED   得分: {:.0f}\n'
                '中灯 YELLOW得分: {:.0f}\n'
                '右灯 GREEN 得分: {:.0f}\n'
                '============================'.format(
                    state,
                    scores[0],
                    scores[1],
                    scores[2]
                )
            )

    def detect_light(self, frame):

        h, w = frame.shape[:2]

        # =====================================================
        # 红绿灯位于摄像头画面的上部区域
        # 使用比例而不是固定像素，机器人距离变化时仍然可用
        # =====================================================
        y1 = int(h * 0.15)
        y2 = int(h * 0.60)

        x1 = int(w * 0.05)
        x2 = int(w * 0.95)

        roi = frame[y1:y2, x1:x2]

        hsv = cv2.cvtColor(
            roi,
            cv2.COLOR_BGR2HSV
        )

        # =====================================================
        # 只寻找“亮起来”的高饱和度、高亮度灯
        # =====================================================

        # 红色跨越 HSV 0°
        red1 = cv2.inRange(
            hsv,
            np.array([0, 130, 170]),
            np.array([12, 255, 255])
        )

        red2 = cv2.inRange(
            hsv,
            np.array([168, 130, 170]),
            np.array([179, 255, 255])
        )

        red_mask = cv2.bitwise_or(
            red1,
            red2
        )

        yellow_mask = cv2.inRange(
            hsv,
            np.array([13, 130, 170]),
            np.array([38, 255, 255])
        )

        green_mask = cv2.inRange(
            hsv,
            np.array([39, 100, 140]),
            np.array([95, 255, 255])
        )

        kernel = np.ones(
            (3, 3),
            np.uint8
        )

        red_mask = cv2.morphologyEx(
            red_mask,
            cv2.MORPH_OPEN,
            kernel
        )

        yellow_mask = cv2.morphologyEx(
            yellow_mask,
            cv2.MORPH_OPEN,
            kernel
        )

        green_mask = cv2.morphologyEx(
            green_mask,
            cv2.MORPH_OPEN,
            kernel
        )

        # =====================================================
        # 找三个颜色中最明显的亮灯
        # =====================================================

        red = self.best_blob(red_mask)
        yellow = self.best_blob(yellow_mask)
        green = self.best_blob(green_mask)

        candidates = []

        if red is not None:
            candidates.append(
                ('RED', red)
            )

        if yellow is not None:
            candidates.append(
                ('YELLOW', yellow)
            )

        if green is not None:
            candidates.append(
                ('GREEN', green)
            )

        if not candidates:
            return 'UNKNOWN', (0, 0, 0)

        # =====================================================
        # 不只看颜色，还检查横向位置
        #
        # 红灯应该在最左
        # 黄灯应该在中间
        # 绿灯应该在最右
        # =====================================================

        valid = []

        roi_width = roi.shape[1]

        for state, blob in candidates:

            area, cx, cy = blob

            relative_x = cx / float(roi_width)

            position_ok = False

            if state == 'RED':
                position_ok = relative_x < 0.55

            elif state == 'YELLOW':
                position_ok = (
                    0.20 < relative_x < 0.80
                )

            elif state == 'GREEN':
                position_ok = relative_x > 0.45

            if position_ok:
                valid.append(
                    (state, area)
                )

        if not valid:
            return 'UNKNOWN', (
                red[0] if red else 0,
                yellow[0] if yellow else 0,
                green[0] if green else 0
            )

        # 最大亮色区域作为当前亮灯
        state = max(
            valid,
            key=lambda item: item[1]
        )[0]

        scores = (
            red[0] if red else 0,
            yellow[0] if yellow else 0,
            green[0] if green else 0
        )

        return state, scores

    def best_blob(self, mask):

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        best = None

        for contour in contours:

            area = cv2.contourArea(contour)

            # 排除非常小的颜色噪点
            if area < 15:
                continue

            x, y, w, h = cv2.boundingRect(
                contour
            )

            # 红绿灯灯珠基本接近方形/圆形区域
            ratio = w / float(max(h, 1))

            if ratio < 0.35 or ratio > 2.8:
                continue

            cx = x + w / 2.0
            cy = y + h / 2.0

            if best is None or area > best[0]:
                best = (
                    area,
                    cx,
                    cy
                )

        return best


def main(args=None):

    rclpy.init(args=args)

    node = TrafficLightDetection()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
