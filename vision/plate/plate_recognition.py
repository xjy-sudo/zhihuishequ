#!/usr/bin/env python3

import cv2
import re
import time
import numpy as np
import pytesseract
import rclpy

from collections import Counter, deque
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


VALID_PLATES = [
    '闽A.JVV88',
    '皖V.K7T8X',
    '苏A.B8Q62',
]


class PlateRecognition(Node):

    def __init__(self):
        super().__init__('plate_recognition')

        self.bridge = CvBridge()

        self.create_subscription(
            Image,
            '/camera/front_camera/image_raw',
            self.image_callback,
            10
        )

        self.history = deque(maxlen=8)
        self.last_result = ''
        self.last_confidence = 0.0

        # 导航触发控制
        self.recognition_active = False
        self.current_target = ''

        self.create_subscription(
            String,
            '/plate_recognition/command',
            self.command_callback,
            10
        )
        self.last_debug = 0.0

        # 第一次摄像头画面到达时，对车牌识别流程做一次静默预热
        self.ocr_warmed_up = False

        self.get_logger().info('================================')
        self.get_logger().info('车牌识别 V3 启动')
        self.get_logger().info('汽车ROI -> 车牌ROI -> OCR -> 多帧投票')
        self.get_logger().info('================================')

    # --------------------------------------------------
    # 找汽车立牌
    # --------------------------------------------------
    def find_car(self, frame):

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        H, W = frame.shape[:2]

        # 汽车主体蓝色
        mask = cv2.inRange(
            hsv,
            np.array([90, 45, 25]),
            np.array([150, 255, 255])
        )

        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (11, 11)
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel,
            iterations=3
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        candidates = []

        for c in contours:

            x, y, w, h = cv2.boundingRect(c)

            # 汽车在实际摄像头里明显较大
            if w < 100 or h < 80:
                continue

            if w > W * 0.80:
                continue

            if h > H * 0.80:
                continue

            ratio = w / float(h)

            if ratio < 0.8 or ratio > 3.5:
                continue

            candidates.append(
                (w * h, x, y, w, h)
            )

        if not candidates:
            return None

        candidates.sort(reverse=True)

        _, x, y, w, h = candidates[0]

        # 向外扩展，让框覆盖完整汽车立牌
        px = int(w * 0.10)
        py = int(h * 0.12)

        x1 = max(0, x - px)
        y1 = max(0, y - py)
        x2 = min(W, x + w + px)
        y2 = min(H, y + h + py)

        return x1, y1, x2, y2

    # --------------------------------------------------
    # 根据汽车ROI提取车牌区域
    # --------------------------------------------------
    def get_plate_roi(self, frame, car):

        x1, y1, x2, y2 = car

        cw = x2 - x1
        ch = y2 - y1

        # 根据当前比赛汽车素材：
        # 车牌位于汽车中央、约高度65%~82%的区域
        # 比赛汽车素材的车牌固定区域
        px1 = int(x1 + cw * 0.28)
        px2 = int(x1 + cw * 0.72)

        py1 = int(y1 + ch * 0.60)
        py2 = int(y1 + ch * 0.84)

        px1 = max(0, px1)
        py1 = max(0, py1)
        px2 = min(frame.shape[1], px2)
        py2 = min(frame.shape[0], py2)

        return px1, py1, px2, py2

    # --------------------------------------------------
    # OCR文本标准化
    # --------------------------------------------------
    def normalize(self, text):

        text = text.upper()

        text = text.replace(' ', '')
        text = text.replace('.', '')
        text = text.replace('·', '')
        text = text.replace('-', '')
        text = text.replace('_', '')

        return re.sub(
            r'[^A-Z0-9\u4e00-\u9fff]',
            '',
            text
        )

    # --------------------------------------------------
    # 编辑距离
    # --------------------------------------------------
    def distance(self, a, b):

        if not a:
            return len(b)

        if not b:
            return len(a)

        previous = list(range(len(b) + 1))

        for i, ca in enumerate(a, 1):

            current = [i]

            for j, cb in enumerate(b, 1):

                current.append(
                    min(
                        current[-1] + 1,
                        previous[j] + 1,
                        previous[j - 1] +
                        (0 if ca == cb else 1)
                    )
                )

            previous = current

        return previous[-1]

    # --------------------------------------------------
    # 三个合法车牌候选匹配
    # --------------------------------------------------
    def match_plate(self, raw):

        raw = self.normalize(raw)

        # OCR最稳定的是英文字母和数字
        raw_ascii = re.sub(
            r'[^A-Z0-9]',
            '',
            raw
        )

        if len(raw_ascii) < 3:
            return None, 0.0

        best_plate = None
        best_score = 0.0

        for plate in VALID_PLATES:

            target = self.normalize(plate)

            target_ascii = re.sub(
                r'[^A-Z0-9]',
                '',
                target
            )

            d = self.distance(
                raw_ascii,
                target_ascii
            )

            score = 1.0 - (
                d /
                max(
                    len(raw_ascii),
                    len(target_ascii)
                )
            )

            if score > best_score:
                best_score = score
                best_plate = plate

        # 先放宽到0.35，多帧投票会继续过滤
        if best_score < 0.35:
            return None, best_score

        return best_plate, best_score

    # --------------------------------------------------
    # OCR
    # --------------------------------------------------
    def ocr_plate(self, roi):

        if roi.size == 0:
            return '', None, 0.0, None

        enlarged = cv2.resize(
            roi,
            None,
            fx=8,
            fy=8,
            interpolation=cv2.INTER_CUBIC
        )

        gray = cv2.cvtColor(
            enlarged,
            cv2.COLOR_BGR2GRAY
        )

        # 对比度增强
        gray = cv2.equalizeHist(gray)

        _, binary = cv2.threshold(
            gray,
            0,
            255,
            cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )

        inv = cv2.bitwise_not(binary)

        best_raw = ''
        best_plate = None
        best_score = 0.0
        best_img = binary

        # 原图、二值、反色都尝试
        images = [
            enlarged,
            binary,
            inv
        ]

        for img in images:

            for psm in (7, 8, 13):

                raw = pytesseract.image_to_string(
                    img,
                    lang='eng',
                    config=(
                        f'--psm {psm} '
                        '-c tessedit_char_whitelist=ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789'
                    )
                ).strip()

                plate, score = self.match_plate(raw)

                if plate is not None and score > best_score:

                    best_raw = raw
                    best_plate = plate
                    best_score = score

                    if len(img.shape) == 2:
                        best_img = img
                    else:
                        best_img = gray

        return (
            best_raw,
            best_plate,
            best_score,
            best_img
        )

    # --------------------------------------------------
    # 摄像头回调
    # --------------------------------------------------
    def command_callback(self, msg):

        command = msg.data.strip()

        if command.startswith('START:'):

            self.current_target = command.split(':', 1)[1]

            # 每一个车牌点必须重新统计
            self.history.clear()
            self.last_result = ''
            self.last_confidence = 0.0
            self.recognition_active = True

            self.get_logger().info(
                f'▶ {self.current_target}：车牌识别开启'
            )

        elif command == 'STOP':

            if self.recognition_active:

                # --------------------------------------------------
                # STOP兜底：
                # 如果识别期间已经2帧确认，last_result已有结果，
                # 此处不重复打印。
                #
                # 如果只有1帧或未达到2帧，但history里存在真实OCR结果，
                # 则从真实历史结果中选择累计次数最多、平均置信度最高者。
                # --------------------------------------------------
                if not self.last_result and self.history:

                    stats = {}

                    for plate, score in self.history:
                        if plate not in stats:
                            stats[plate] = []

                        stats[plate].append(score)

                    best_plate = None
                    best_count = 0
                    best_confidence = 0.0

                    for plate, scores in stats.items():

                        count = len(scores)
                        avg_confidence = sum(scores) / count

                        if (
                            count > best_count or
                            (
                                count == best_count and
                                avg_confidence > best_confidence
                            )
                        ):
                            best_plate = plate
                            best_count = count
                            best_confidence = avg_confidence

                    if best_plate is not None:

                        self.last_result = best_plate
                        self.last_confidence = best_confidence

                        self.get_logger().info(
                            '================================'
                        )
                        self.get_logger().info(
                            f'{self.current_target} 车牌识别成功'
                        )
                        self.get_logger().info(
                            '目标类别：车牌'
                        )
                        self.get_logger().info(
                            f'识别内容：{self.last_result}'
                        )
                        self.get_logger().info(
                            f'置信度：{self.last_confidence:.2f}'
                        )
                        self.get_logger().info(
                            '================================'
                        )

                # 没有任何真实有效OCR结果时静默结束，
                # 不打印“识别失败/未识别到有效车牌”
                self.get_logger().info(
                    f'■ {self.current_target}：车牌识别关闭'
                )

            self.recognition_active = False
            self.current_target = ''
            self.history.clear()

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        output = frame.copy()

        # --------------------------------------------------
        # 首帧静默预热
        # 只运行一次，不保存结果、不计入正式识别
        # --------------------------------------------------
        if not self.ocr_warmed_up:

            self.get_logger().info(
                '车牌识别首次预热中...'
            )

            car_warm = self.find_car(frame)

            if car_warm is not None:
                wx1, wy1, wx2, wy2 = car_warm

                plate_box_warm = self.get_plate_roi(
                    frame,
                    car_warm
                )

                wpx1, wpy1, wpx2, wpy2 = plate_box_warm

                roi_warm = frame[
                    wpy1:wpy2,
                    wpx1:wpx2
                ]

                if roi_warm.size > 0:
                    self.ocr_plate(roi_warm)

            self.ocr_warmed_up = True

            self.get_logger().info(
                '✓ 车牌识别预热完成'
            )

        # 未收到导航 START 时只显示实时摄像头
        if not self.recognition_active:
            cv2.imshow(
                'Plate Recognition',
                output
            )
            cv2.waitKey(1)
            return

        car = self.find_car(frame)

        if car is None:

            now = time.time()

            if now - self.last_debug > 1.0:
                self.get_logger().info(
                    '未找到汽车立牌'
                )
                self.last_debug = now

            cv2.imshow(
                'Plate Recognition',
                output
            )

            cv2.waitKey(1)
            return

        x1, y1, x2, y2 = car

        # 绿色框：整辆汽车
        cv2.rectangle(
            output,
            (x1, y1),
            (x2, y2),
            (0, 255, 0),
            2
        )

        plate_box = self.get_plate_roi(
            frame,
            car
        )

        px1, py1, px2, py2 = plate_box

        # 黄色框：实际送入OCR的区域
        cv2.rectangle(
            output,
            (px1, py1),
            (px2, py2),
            (0, 255, 255),
            2
        )

        roi = frame[
            py1:py2,
            px1:px2
        ]

        raw, plate, score, binary = \
            self.ocr_plate(roi)

        now = time.time()

        # 正式比赛版：关闭逐帧OCR调试输出
        self.last_debug = now

        if plate is not None:

            self.history.append(
                (plate, score)
            )

        # --------------------------------------------------
        # 多帧累计确认
        # 同一车牌累计成功2帧后立即确认并同步输出
        # --------------------------------------------------
        if plate is not None:

            confidence = score

            ascii_plate = re.sub(
                r'[^\x00-\x7F]',
                '',
                plate
            )

            cv2.putText(
                output,
                f'{ascii_plate} {confidence:.2f}',
                (x1, max(30, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 255, 0),
                2
            )

            # 尚未确认结果时，统计当前观察点累计识别结果
            if not self.last_result:

                plate_records = [
                    item for item in self.history
                    if item[0] == plate
                ]

                # 同一车牌累计识别成功2帧即确认
                if len(plate_records) >= 2:

                    self.last_result = plate

                    # 使用该车牌所有成功帧的平均置信度
                    self.last_confidence = sum(
                        item[1] for item in plate_records
                    ) / len(plate_records)

                    self.get_logger().info(
                        '================================'
                    )
                    self.get_logger().info(
                        f'{self.current_target} 车牌识别成功'
                    )
                    self.get_logger().info(
                        '目标类别：车牌'
                    )
                    self.get_logger().info(
                        f'识别内容：{self.last_result}'
                    )
                    self.get_logger().info(
                        f'置信度：{self.last_confidence:.2f}'
                    )
                    self.get_logger().info(
                        '================================'
                    )

        cv2.imshow(
            'Plate Recognition',
            output
        )

        if binary is not None:
            cv2.imshow(
                'Plate OCR ROI',
                binary
            )

        cv2.waitKey(1)


def main():

    rclpy.init()

    node = PlateRecognition()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
