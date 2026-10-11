#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2


class YoloCameraTest(Node):

    def __init__(self):
        super().__init__('yolo_camera_test')

        self.bridge = CvBridge()

        model_path = (
            '/home/xjy/smart_community_ws/'
            'yolo_runs/person_v2/weights/best.pt'
        )

        self.model = YOLO(model_path)

        self.subscription = self.create_subscription(
            Image,
            '/camera/front_camera/image_raw',
            self.image_callback,
            10
        )

        # 导航触发控制
        self.recognition_active = False
        self.current_target = ''

        # YOLO首次推理预热
        self.model_warmed_up = False

        # 当前观察点已经识别到的人
        # key = 人物名称，value = 最高置信度
        self.current_people = {}

        # 整个街区统计
        # 同一个人物只统计一次
        self.all_people = {}

        self.command_subscription = self.create_subscription(
            String,
            '/person_recognition/command',
            self.command_callback,
            10
        )

        self.get_logger().info('=================================')
        self.get_logger().info('YOLO人物识别测试已启动')
        self.get_logger().info(f'模型：{model_path}')
        self.get_logger().info('绿色 = 社区人员')
        self.get_logger().info('红色 = 非社区人员')
        self.get_logger().info('=================================')

    def get_area_name(self, target):

        area_map = {
            'PERSON_1': 'A街区',
            'PERSON_2': 'A街区',
            'PERSON_3_LEFT': 'A街区',

            'PERSON_3_RIGHT': 'B街区',
            'PERSON_RIGHT': 'B街区',
            'PERSON_4': 'B街区',
        }

        return area_map.get(target, target)

    def command_callback(self, msg):

        command = msg.data.strip()

        if command.startswith('START:'):

            parts = command.split(':')

            self.current_target = (
                parts[1] if len(parts) >= 2 else 'PERSON'
            )
            # 新观察点必须清空，防止上一观察点污染
            self.current_people.clear()

            self.recognition_active = True

            area_name = self.get_area_name(
                self.current_target
            )

            self.get_logger().info(
                f'▶ {area_name}：人物识别开启'
            )

        elif command == 'STOP':

            if self.recognition_active:

                area_name = self.get_area_name(
                    self.current_target
                )

                self.get_logger().info(
                    f'========== {area_name} 人物识别结果 =========='
                )

                if not self.current_people:

                    self.get_logger().info(
                        '本观察点未识别到人物'
                    )

                else:
                    # 按名称排序，保证终端结果稳定
                    people = sorted(
                        self.current_people.items()
                    )

                    for name, confidence in people:

                        if name.startswith('community_'):
                            category = '社区人员'
                        else:
                            category = '非社区人员'

                        self.get_logger().info(
                            f'目标类别：{category} | '
                            f'识别内容：{name} | '
                            f'置信度：{confidence:.2f}'
                        )

                self.get_logger().info(
                    f'本观察点人数：{len(self.current_people)}'
                )

                community_count = sum(
                    1 for name in self.all_people
                    if name.startswith('community_')
                )

                noncommunity_count = (
                    len(self.all_people) -
                    community_count
                )

                self.get_logger().info(
                    f'街区累计：{len(self.all_people)} 人 | '
                    f'社区人员 {community_count} | '
                    f'非社区人员 {noncommunity_count}'
                )

                self.get_logger().info(
                    '=========================================='
                )

            self.recognition_active = False
            self.current_target = ''
            self.current_people.clear()

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        # 第一次收到摄像头画面时静默预热YOLO
        # 不统计人物、不画框，不影响实时摄像头显示
        if not self.model_warmed_up:

            self.get_logger().info(
                'YOLO首次推理预热中...'
            )

            self.model.predict(
                source=frame,
                imgsz=640,
                conf=0.40,
                device='cpu',
                verbose=False
            )

            self.model_warmed_up = True

            self.get_logger().info(
                '✓ YOLO预热完成'
            )

        # 没有收到导航START时继续显示实时摄像头
        if not self.recognition_active:

            cv2.imshow(
                'YOLO Person Recognition',
                frame
            )

            cv2.waitKey(1)
            return

        results = self.model.predict(
            source=frame,
            imgsz=640,
            conf=0.40,
            device='cpu',
            verbose=False
        )

        output = frame.copy()

        for result in results:

            if result.boxes is None:
                continue

            for box in result.boxes:

                cls_id = int(box.cls[0])
                confidence = float(box.conf[0])

                name = self.model.names[cls_id]

                # 当前观察点：同一人物保留最高置信度
                old_conf = self.current_people.get(
                    name,
                    0.0
                )

                if confidence > old_conf:
                    self.current_people[name] = confidence

                # 整个街区：同一人物只统计一次
                old_all_conf = self.all_people.get(
                    name,
                    0.0
                )

                if confidence > old_all_conf:
                    self.all_people[name] = confidence

                x1, y1, x2, y2 = map(
                    int,
                    box.xyxy[0].tolist()
                )

                if name.startswith('community_'):
                    color = (0, 255, 0)
                    category = 'COMMUNITY'
                else:
                    color = (0, 0, 255)
                    category = 'NON-COMMUNITY'

                cv2.rectangle(
                    output,
                    (x1, y1),
                    (x2, y2),
                    color,
                    2
                )

                label = f'{name} {confidence:.2f}'

                cv2.putText(
                    output,
                    label,
                    (x1, max(25, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    color,
                    2
                )

        cv2.imshow(
            'YOLO Person Recognition',
            output
        )

        cv2.waitKey(1)


def main():

    rclpy.init()

    node = YoloCameraTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
