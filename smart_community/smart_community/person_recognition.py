#!/usr/bin/env python3

import cv2
import numpy as np
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class PersonRecognition(Node):

    def __init__(self):
        super().__init__('person_recognition')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/front_camera/image_raw',
            self.image_callback,
            10
        )

        # 人物识别只允许由导航程序触发
        self.command_sub = self.create_subscription(
            String,
            '/person_recognition/command',
            self.command_callback,
            10
        )

        self.recognition_active = False
        self.current_target = ''
        self.expected_count = 0

        # 当前识别点保存到的最佳结果
        self.best_results = []

        self.base = (
            Path.home()
            / 'smart_community_ws/src/smart_community/models'
        )

        # ORB
        self.orb = cv2.ORB_create(
            nfeatures=1500,
            scaleFactor=1.2,
            nlevels=8
        )

        self.matcher = cv2.BFMatcher(
            cv2.NORM_HAMMING,
            crossCheck=True
        )

        self.templates = []

        self.load_templates()

        # 已经统计过的具体人物
        self.seen_people = set()

        # 防止终端每一帧疯狂重复打印
        self.last_result = None

        self.get_logger().info(
            f'人物识别节点启动，模板数量={len(self.templates)}'
        )
        self.get_logger().info(
            '订阅摄像头：/camera/front_camera/image_raw'
        )

    # ==========================================================
    # 加载18个人物模板
    # ==========================================================
    def load_templates(self):

        items = []

        for i in range(1, 17):
            items.append((
                f'community_{i:02d}',
                '社区人员',
                self.base
                / f'person_community_{i:02d}'
                / 'materials/textures'
                / f'community_{i:02d}.png'
            ))

        for i in range(1, 3):
            items.append((
                f'noncommunity_{i:02d}',
                '非社区人员',
                self.base
                / f'person_noncommunity_{i:02d}'
                / 'materials/textures'
                / f'noncommunity_{i:02d}.png'
            ))

        for name, category, path in items:

            img = cv2.imread(
                str(path),
                cv2.IMREAD_UNCHANGED
            )

            if img is None:
                self.get_logger().error(
                    f'模板读取失败：{path}'
                )
                continue

            # 去掉alpha通道
            if img.ndim == 3 and img.shape[2] == 4:
                img = cv2.cvtColor(
                    img,
                    cv2.COLOR_BGRA2BGR
                )

            gray = cv2.cvtColor(
                img,
                cv2.COLOR_BGR2GRAY
            )

            kp, des = self.orb.detectAndCompute(
                gray,
                None
            )

            if des is None:
                self.get_logger().warning(
                    f'{name} 没有检测到ORB特征'
                )
                continue

            self.templates.append({
                'name': name,
                'category': category,
                'image': img,
                'kp': kp,
                'des': des
            })

            self.get_logger().info(
                f'✓ 加载模板 {category} {name} '
                f'特征点={len(kp)}'
            )

    # ==========================================================
    # 找灰色人物立牌
    # ==========================================================
    def detect_boards(self, frame):
        """检测 Gazebo 中竖直人物立牌"""

        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )

        # 人物立牌与道路/天空之间存在明显边缘
        edges = cv2.Canny(
            gray,
            35,
            100
        )

        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (3, 5)
        )

        edges = cv2.morphologyEx(
            edges,
            cv2.MORPH_CLOSE,
            kernel,
            iterations=2
        )

        contours, _ = cv2.findContours(
            edges,
            cv2.RETR_LIST,
            cv2.CHAIN_APPROX_SIMPLE
        )

        h_img, w_img = frame.shape[:2]

        candidates = []

        for cnt in contours:

            x, y, w, h = cv2.boundingRect(cnt)

            area = w * h

            # 过滤极小噪声
            if area < 700:
                continue

            # 人物立牌必须具有一定高度和宽度
            if h < 55 or w < 15:
                continue

            # 人物立牌明显高于宽
            ratio = h / float(max(w, 1))

            if ratio < 1.35:
                continue

            # 排除建筑物等超大目标
            if w > w_img * 0.22:
                continue

            if h > h_img * 0.80:
                continue

            # 人物一般位于图像下部，排除天空区域
            if y + h < h_img * 0.48:
                continue

            candidates.append(
                (x, y, w, h)
            )

        # ----------------------------------------------------
        # 去掉同一个立牌产生的嵌套框
        # ----------------------------------------------------
        candidates.sort(
            key=lambda b: b[2] * b[3],
            reverse=True
        )

        boxes = []

        for box in candidates:

            x, y, w, h = box

            cx = x + w / 2.0
            cy = y + h / 2.0

            duplicate = False

            for bx, by, bw, bh in boxes:

                bcx = bx + bw / 2.0
                bcy = by + bh / 2.0

                if (
                    abs(cx - bcx) < max(w, bw) * 0.35
                    and
                    abs(cy - bcy) < max(h, bh) * 0.35
                ):
                    duplicate = True
                    break

            if not duplicate:
                boxes.append(box)

        # ----------------------------------------------------
        # 如果候选框多于当前识别点期望人数：
        # 优先保留面积较大的立牌
        # ----------------------------------------------------
        if (
            self.expected_count > 0
            and len(boxes) > self.expected_count
        ):
            boxes = sorted(
                boxes,
                key=lambda b: b[2] * b[3],
                reverse=True
            )[:self.expected_count]

        # 最终从左向右排列
        boxes.sort(
            key=lambda b: b[0]
        )

        return boxes, edges

    # ==========================================================
    # 与18张人物模板进行ORB匹配
    # ==========================================================
    def recognize_person(self, roi):

        if roi.size == 0:
            return None

        gray = cv2.cvtColor(
            roi,
            cv2.COLOR_BGR2GRAY
        )

        kp, des = self.orb.detectAndCompute(
            gray,
            None
        )

        if des is None or len(kp) < 4:
            return None

        best = None
        best_score = 0.0
        best_good = 0

        for template in self.templates:

            try:
                matches = self.matcher.match(
                    template['des'],
                    des
                )
            except cv2.error:
                continue

            if not matches:
                continue

            matches = sorted(
                matches,
                key=lambda m: m.distance
            )

            # 好匹配数量
            good = [
                m for m in matches
                if m.distance < 65
            ]

            good_count = len(good)

            # 同时考虑匹配数量和距离
            if good_count > 0:
                mean_distance = np.mean(
                    [m.distance for m in good]
                )
            else:
                mean_distance = 100.0

            # 第一版置信度：
            # 好匹配越多、距离越小，置信度越高
            count_score = min(
                good_count / 25.0,
                1.0
            )

            distance_score = max(
                0.0,
                1.0 - mean_distance / 100.0
            )

            score = (
                0.65 * count_score
                + 0.35 * distance_score
            )

            if score > best_score:
                best_score = score
                best_good = good_count
                best = template

        if best is None:
            return None

        # 第一阶段阈值先稍微放宽，观察真实结果后再调
        if best_good < 3:
            return None

        return (
            best['name'],
            best['category'],
            float(best_score),
            best_good
        )

    # ==========================================================
    # 摄像头回调
    # ==========================================================
    def command_callback(self, msg):

        command = msg.data.strip()

        if command == 'STOP':

            if self.recognition_active:

                # 识别结束时只输出一次最终统计
                community_count = sum(
                    1 for r in self.best_results
                    if r[5] == '社区人员'
                )

                noncommunity_count = sum(
                    1 for r in self.best_results
                    if r[5] == '非社区人员'
                )

                total = (
                    community_count
                    + noncommunity_count
                )

                self.get_logger().info(
                    f'========== {self.current_target} 识别结果 =========='
                )

                self.get_logger().info(
                    f'社区人员：{community_count}  '
                    f'非社区人员：{noncommunity_count}  '
                    f'总计：{total}人'
                )

                self.get_logger().info(
                    '========================================'
                )

                # 加入整个街区去重统计
                for r in self.best_results:
                    self.seen_people.add(r[4])

            self.recognition_active = False
            self.current_target = ''
            self.expected_count = 0
            self.last_result = None

            # 注意：best_results 暂时不清空，
            # 这样 STOP 后窗口还能保留最后一次框选
            return

        if command.startswith('START:'):

            parts = command.split(':')

            if len(parts) != 3:
                self.get_logger().warning(
                    f'无法解析人物识别命令：{command}'
                )
                return

            try:
                expected = int(parts[2])
            except ValueError:
                self.get_logger().warning(
                    f'人数参数错误：{command}'
                )
                return

            self.current_target = parts[1]
            self.expected_count = expected
            self.recognition_active = True

            # 新识别点重新开始
            self.best_results = []
            self.last_result = None

            self.get_logger().info(
                f'▶ {self.current_target}：人物识别开启，'
                f'期望人数={self.expected_count}'
            )

    def image_callback(self, msg):

        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().error(
                f'图像转换失败：{e}'
            )
            return

        output = frame.copy()

        # 未到人物识别点时禁止进行人物识别
        if not self.recognition_active:
            cv2.imshow(
                'Person Recognition',
                output
            )
            cv2.waitKey(1)
            return

        boxes, mask = self.detect_boards(
            frame
        )

        current_results = []

        for x, y, w, h in boxes:

            # 稍微缩进，减少相邻立牌边缘影响
            margin_x = max(
                2,
                int(w * 0.05)
            )

            margin_y = max(
                2,
                int(h * 0.03)
            )

            x1 = max(
                0,
                x + margin_x
            )

            y1 = max(
                0,
                y + margin_y
            )

            x2 = min(
                frame.shape[1],
                x + w - margin_x
            )

            y2 = min(
                frame.shape[0],
                y + h - margin_y
            )

            roi = frame[
                y1:y2,
                x1:x2
            ]

            result = self.recognize_person(
                roi
            )

            if result is None:
                continue

            name, category, confidence, good = result

            current_results.append(
                (
                    x,
                    y,
                    w,
                    h,
                    name,
                    category,
                    confidence,
                    good
                )
            )

        # 从左向右排序
        current_results.sort(
            key=lambda r: r[0]
        )

        # 防止同一帧把同一个身份识别两次
        used_names = set()
        filtered_results = []

        for result in current_results:

            name = result[4]

            if name in used_names:
                continue

            used_names.add(name)
            filtered_results.append(result)

        # 保存当前识别点检测数量最多的一帧
        # 数量相同时，优先保存总置信度更高的一帧
        current_score = sum(
            r[6] for r in filtered_results
        )

        best_score = sum(
            r[6] for r in self.best_results
        )

        if (
            len(filtered_results) > len(self.best_results)
            or
            (
                len(filtered_results) == len(self.best_results)
                and current_score > best_score
            )
        ):
            self.best_results = list(
                filtered_results
            )

        # 达到期望人数以后优先显示稳定的最佳结果
        display_results = (
            self.best_results
            if self.best_results
            else filtered_results
        )

        # ======================================================
        # 图像输出
        # ======================================================
        for (
            x,
            y,
            w,
            h,
            name,
            category,
            confidence,
            good
        ) in display_results:

            box_color = (
                (0, 255, 0)
                if category == '社区人员'
                else (0, 0, 255)
            )

            cv2.rectangle(
                output,
                (x, y),
                (x + w, y + h),
                box_color,
                2
            )

            # OpenCV默认字体不方便直接显示中文，
            # 所以画面使用英文类别，终端使用中文类别
            short_category = (
                'COMMUNITY'
                if category == '社区人员'
                else 'NONCOMMUNITY'
            )

            label = (
                f'{short_category} '
                f'{name} '
                f'{confidence:.2f}'
            )

            label_y = max(
                20,
                y - 8
            )

            cv2.putText(
                output,
                label,
                (x, label_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                box_color,
                1,
                cv2.LINE_AA
            )

        # 终端结果在收到 STOP 时统一输出一次

        cv2.imshow(
            'Person Recognition',
            output
        )

        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = PersonRecognition()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
