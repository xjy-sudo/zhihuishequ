#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener


class PatrolNavigation(Node):

    def __init__(self):
        super().__init__('patrol_navigation')

        # 使用 Gazebo 仿真时间
        self.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])

        # 从第几个导航点开始，默认从1开始
        self.declare_parameter('start_index', 1)
        self.start_index = int(
            self.get_parameter('start_index').value
        )

        # ==========================================================
        # 13个导航点
        # 格式：(名称, x, y, Nav2最终目标朝向)
        #
        # 0°    向右
        # 90°   向上
        # 180°  向左
        # -90°  向下
        # ==========================================================
        self.waypoints = [
            ('LIGHT_1',   1.600, -0.300,   90.0),
            ('PERSON_1',  1.600,  1.054,   90.0),
            ('TURN_1',    1.600,  1.640,  180.0),
            ('PERSON_2',  1.100,  1.640,  180.0),
            ('TURN_2',    0.450,  1.640,  -90.0),
            ('PERSON_3',  0.450,  0.850,  -90.0),
            ('TURN_3',    0.450,  0.040,  180.0),
            ('PERSON_4',  0.010,  0.040,  180.0),
            ('LIGHT_2',  -0.350,  0.040,  180.0),
            ('TURN_4',   -1.570,  0.040,  -90.0),
            ('PLATE_1',  -1.570, -1.100,  -90.0),
            ('PLATE_2',  -1.030, -1.100,    0.0),
            ('PLATE_3',   -0.460, -1.100,    0.0),
            ('RETURN_1',    1.600, -1.100,  -90.0),
            ('P0_RETURN',   1.645, -1.522,  -90.0),
        ]

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # 人物识别触发命令
        self.person_recognition_pub = self.create_publisher(
            String,
            '/person_recognition/command',
            10
        )

        self.plate_recognition_pub = self.create_publisher(
            String,
            '/plate_recognition/command',
            10
        )

        # 红绿灯视觉识别结果
        self.traffic_light_state = 'UNKNOWN'

        self.traffic_light_sub = self.create_subscription(
            String,
            '/traffic_light/detected_state',
            self.traffic_light_callback,
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.current_index = self.start_index - 1

        self.get_logger().info('====================================')
        self.get_logger().info('新版智慧社区13点巡视程序启动')
        self.get_logger().info(
            f'从第 {self.start_index} 个导航点开始'
        )
        self.get_logger().info('====================================')

        self.start_timer = self.create_timer(
            1.0,
            self.start_navigation
        )

    def start_navigation(self):
        self.start_timer.cancel()

        self.get_logger().info('等待 Nav2...')

        if not self.nav_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error(
                'Nav2 navigate_to_pose 未连接'
            )
            return

        self.get_logger().info('Nav2 已连接')
        self.send_next_goal()

    # ==========================================================
    # 发送 Nav2 导航目标
    # ==========================================================
    def send_next_goal(self):

        if self.current_index >= len(self.waypoints):
            self.get_logger().info('====================================')
            self.get_logger().info('✓ 13个导航点全部完成')
            self.get_logger().info('====================================')
            self.stop_robot()
            return

        name, x, y, yaw_deg = self.waypoints[
            self.current_index
        ]

        self.get_logger().info(
            f'>>> [{self.current_index + 1}] {name} '
            f'x={x:.3f}, y={y:.3f}, '
            f'目标朝向={yaw_deg:.1f}°'
        )

        goal = NavigateToPose.Goal()

        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0

        # Nav2只负责到达(x, y)，最终朝向由程序转向完成
        # 不在发送Goal时读取TF，避免TF时间跳变造成长时间等待
        # yaw_goal_tolerance=3.14，因此这里的朝向不会限制最终到点
        yaw_rad = math.radians(yaw_deg)

        goal.pose.pose.orientation.z = math.sin(
            yaw_rad / 2.0
        )
        goal.pose.pose.orientation.w = math.cos(
            yaw_rad / 2.0
        )

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if goal_handle is None:
            self.get_logger().error(
                'Nav2 没有返回 GoalHandle'
            )
            return

        if not goal_handle.accepted:
            name = self.waypoints[
                self.current_index
            ][0]

            self.get_logger().error(
                f'✗ {name} 导航目标被拒绝'
            )
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self.navigation_result_callback
        )

    def navigation_result_callback(self, future):

        result = future.result()
        status = result.status

        name = self.waypoints[
            self.current_index
        ][0]

        # ROS2 action STATUS_SUCCEEDED = 4
        if status != 4:
            self.get_logger().error(
                f'✗ {name} 导航失败 status={status}'
            )
            self.stop_robot()
            return

        current_yaw = self.get_current_yaw()

        if current_yaw is not None:
            self.get_logger().info(
                f'✓ 到达 {name}，当前方向={current_yaw:.1f}°'
            )
        else:
            self.get_logger().info(
                f'✓ 到达 {name}'
            )

        # 红绿灯点：精确校准摄像头朝向后再判断灯色
        if name in ('LIGHT_1', 'LIGHT_2'):
            target_yaw = self.waypoints[self.current_index][3]

            self.get_logger().info(
                f'🚦 {name}：开始红绿灯精确朝向校准，'
                f'目标={target_yaw:.1f}°'
            )

            if not self.rotate_light_yaw(target_yaw):
                self.get_logger().error(
                    f'✗ {name} 红绿灯朝向校准失败'
                )
                self.stop_robot()
                return

            self.get_logger().info(
                f'✓ {name}：朝向校准完成，开始判断红绿灯'
            )

            self.execute_action(name)
            return

        # 其他导航点：最终朝向由程序自己精确校准
        self.stop_robot()

        target_yaw = self.waypoints[
            self.current_index
        ][3]

        self.get_logger().info(
            f'开始 {name} 普通朝向调整：'
            f'目标={target_yaw:.1f}°'
        )

        if not self.rotate_normal(target_yaw):
            self.get_logger().error(
                f'✗ {name} 普通朝向调整失败'
            )
            self.stop_robot()
            return

        # 朝向准确后，再执行该点对应任务
        self.execute_action(name)

    # ==========================================================
    # 每个点到达后的动作
    # ==========================================================
    def execute_action(self, name):

        if name == 'LIGHT_1':
            self.wait_for_green_light('LIGHT_1')
            self.finish_current_point()
            return

        if name == 'PERSON_1':
            # 道路方向90° → 左转看人偶180° → 复位90°
            self.get_logger().info(
                '👤 PERSON_1：转到180°识别人偶'
            )
            self.rotate_to_yaw(180.0)
            self.trigger_person_recognition('PERSON_1', 4, 3.0)

            self.get_logger().info(
                'PERSON_1：绝对复位90°'
            )
            self.rotate_normal(90.0)

            self.finish_current_point()
            return

        if name == 'TURN_1':
            self.get_logger().info(
                '↩ TURN_1：到达方向90°，开始转向180°'
            )

            self.rotate_normal(180.0)

            self.get_logger().info(
                '✓ TURN_1：已转到180°，准备前往PERSON_2'
            )

            self.finish_current_point()
            return

        if name == 'PERSON_2':
            # 道路方向180° → 左转看人偶-90° → 复位180°
            self.get_logger().info(
                '👤 PERSON_2：转到-90°识别人偶'
            )
            self.rotate_to_yaw(-90.0)
            self.trigger_person_recognition('PERSON_2', 2, 3.0)

            self.get_logger().info(
                'PERSON_2：绝对复位180°'
            )
            self.rotate_normal(180.0)

            self.finish_current_point()
            return

        if name == 'TURN_2':
            self.get_logger().info(
                '↩ TURN_2：程序转向完成，当前目标-90°'
            )
            self.finish_current_point()
            return

        if name == 'PERSON_3':
            # 道路方向-90°
            # 左侧 = 0°
            # 右侧 = 180°
            self.get_logger().info(
                '👤 PERSON_3：转到0°识别第一侧'
            )
            self.rotate_to_yaw(0.0)
            self.trigger_person_recognition(
                'PERSON_3_LEFT', 2, 3.0
            )

            self.get_logger().info(
                'PERSON_3：绝对复位-90°'
            )
            self.rotate_normal(-90.0)

            self.get_logger().info(
                '👤 PERSON_3：转到180°识别第二侧'
            )
            self.rotate_to_yaw(180.0)
            self.trigger_person_recognition(
                'PERSON_3_RIGHT', 5, 3.0
            )

            self.get_logger().info(
                'PERSON_3：再次绝对复位-90°'
            )
            self.rotate_normal(-90.0)

            self.finish_current_point()
            return

        if name == 'TURN_3':
            self.get_logger().info(
                '↩ TURN_3：程序转向完成，当前目标180°'
            )

            # TURN_3 -> PERSON_4距离很短
            # 不再交给Nav2，直接使用TF位置闭环直行
            if self.direct_turn3_to_person4():

                # 当前实际已经到达PERSON_4
                self.current_index += 1

                self.get_logger().info(
                    '👤 PERSON_4：已到达观察位置，转到90°识别人偶'
                )

                # 到达位置时仍保持约180°
                # 使用原来的识别转向逻辑
                self.rotate_to_yaw(90.0)

                self.trigger_person_recognition(
                    'PERSON_4',
                    3,
                    3.0
                )

                self.get_logger().info(
                    'PERSON_4：绝对复位180°'
                )
                self.rotate_normal(180.0)

                # PERSON_4任务完成
                self.finish_current_point()

            return

        if name == 'PERSON_4':
            # 道路方向180° → 右转看人偶90° → 复位180°
            self.get_logger().info(
                '👤 PERSON_4：转到90°识别人偶'
            )
            self.rotate_to_yaw(90.0)
            self.trigger_person_recognition('PERSON_4', 3, 3.0)

            self.get_logger().info(
                'PERSON_4：绝对复位180°'
            )
            self.rotate_normal(180.0)

            self.finish_current_point()
            return

        if name == 'LIGHT_2':
            self.wait_for_green_light('LIGHT_2')
            self.finish_current_point()
            return

        if name == 'TURN_4':
            self.get_logger().info(
                '↩ TURN_4：程序转向完成，当前目标-90°'
            )
            self.finish_current_point()
            return

        if name == 'PLATE_1':
            # 道路方向-90°，无需额外转向
            self.get_logger().info(
                '🚗 PLATE_1：保持-90°识别'
            )
            self.trigger_plate_recognition('PLATE_1', 7.0)

            # PLATE_2位于PLATE_1右侧
            # 识别结束后先转到0°，避免Nav2短距离横向导航时左右摆动
            self.get_logger().info(
                'PLATE_1：识别完成，转向0°准备前往PLATE_2'
            )
            self.rotate_normal(0.0)

            self.finish_current_point()
            return

        if name == 'PLATE_2':
            # 道路方向0° → 右转看车牌-90° → 复位0°
            self.get_logger().info(
                '🚗 PLATE_2：转到-90°识别'
            )
            self.rotate_to_yaw(-90.0)
            self.trigger_plate_recognition('PLATE_2', 7.0)

            self.get_logger().info(
                'PLATE_2：绝对复位0°'
            )
            self.rotate_normal(0.0)

            self.finish_current_point()
            return

        if name == 'PLATE_3':
            # 道路方向0° → 右转看车牌-90° → 复位0°
            self.get_logger().info(
                '🚗 PLATE_3：转到-90°识别'
            )
            self.rotate_to_yaw(-90.0)
            self.trigger_plate_recognition('PLATE_3', 7.0)

            self.get_logger().info(
                'PLATE_3：绝对复位0°'
            )
            self.rotate_normal(0.0)

            self.finish_current_point()
            return

        if name == 'RETURN_1':
            self.get_logger().info(
                '↩ RETURN_1：已到达返航转折点，准备直线返回P0'
            )

            # 最后一段由程序直接闭环返航
            # 不再调用 finish_current_point()
            # 因此不会再把 P0_RETURN 发送给 Nav2
            self.direct_return_to_p0()
            return

        if name == 'P0_RETURN':
            # 已由统一最终朝向校准保持 -90°
            self.get_logger().info(
                '🏁 P0_RETURN：已返回P0并复位到-90°，全部巡视任务完成'
            )
            self.finish_current_point()
            return

        self.finish_current_point()

    # ==========================================================
    # 红绿灯视觉识别
    # ==========================================================
    def traffic_light_callback(self, msg):
        self.traffic_light_state = msg.data.strip().upper()

    def wait_for_green_light(self, target_name):

        self.stop_robot()

        self.get_logger().info(
            f'🚦 {target_name}：等待绿灯，当前状态={self.traffic_light_state}'
        )

        last_state = None

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.05
            )

            state = self.traffic_light_state

            if state != last_state:
                self.get_logger().info(
                    f'🚦 {target_name}：识别状态 = {state}'
                )
                last_state = state

            if state == 'GREEN':
                self.get_logger().info(
                    f'✓ {target_name}：检测到绿灯，允许通行'
                )
                self.stop_robot()
                return

            # RED / YELLOW / UNKNOWN 均保持停车
            self.stop_robot()
            time.sleep(0.05)

    def trigger_person_recognition(self, target_name, expected_count, seconds=3.0):
        """转向完成后触发一次人物识别"""

        self.stop_robot()

        msg = String()
        msg.data = f'START:{target_name}:{expected_count}'
        self.person_recognition_pub.publish(msg)

        self.get_logger().info(
            f'📷 {target_name}：开始人物识别，期望人数={expected_count}'
        )

        end_time = time.time() + seconds

        while time.time() < end_time and rclpy.ok():
            self.stop_robot()
            rclpy.spin_once(
                self,
                timeout_sec=0.05
            )

        msg = String()
        msg.data = 'STOP'
        self.person_recognition_pub.publish(msg)

        self.stop_robot()

        self.get_logger().info(
            f'✓ {target_name}：人物识别结束'
        )

    def trigger_plate_recognition(self, target_name, seconds=3.0):

        self.stop_robot()

        msg = String()
        msg.data = f'START:{target_name}'
        self.plate_recognition_pub.publish(msg)

        self.get_logger().info(
            f'🚗 {target_name}：开始车牌识别'
        )

        end_time = time.time() + seconds

        while time.time() < end_time and rclpy.ok():
            self.stop_robot()
            rclpy.spin_once(
                self,
                timeout_sec=0.05
            )

        msg = String()
        msg.data = 'STOP'
        self.plate_recognition_pub.publish(msg)

        self.stop_robot()

        self.get_logger().info(
            f'✓ {target_name}：车牌识别结束'
        )

    def wait_for_recognition(self, target_name, seconds):

        self.stop_robot()

        self.get_logger().info(
            f'📷 {target_name}：停留 {seconds:.1f} 秒'
        )

        end_time = time.time() + seconds

        while time.time() < end_time and rclpy.ok():
            rclpy.spin_once(
                self,
                timeout_sec=0.05
            )

        self.get_logger().info(
            f'✓ {target_name}：停留完成'
        )

    # ==========================================================
    # 获取当前 map -> base_footprint 位置
    # ==========================================================
    def get_current_position(self):

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time()
            )

            return (
                transform.transform.translation.x,
                transform.transform.translation.y
            )

        except Exception as e:
            self.get_logger().warning(
                f'读取当前位置失败：{e}'
            )
            return None

    # ==========================================================
    # 获取当前 map -> base_footprint 朝向
    # ==========================================================
    def get_current_yaw(self):

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time()
            )

            q = transform.transform.rotation

            siny_cosp = 2.0 * (
                q.w * q.z +
                q.x * q.y
            )

            cosy_cosp = 1.0 - 2.0 * (
                q.y * q.y +
                q.z * q.z
            )

            yaw = math.atan2(
                siny_cosp,
                cosy_cosp
            )

            return math.degrees(yaw)

        except Exception as e:
            self.get_logger().warning(
                f'读取当前方向失败：{e}'
            )
            return None

    # ==========================================================
    # 临时相对旋转
    # 正数 = 左转
    # 负数 = 右转
    # ==========================================================
    def rotate_normal(self, target_yaw):
        """普通绝对角度转弯：进入±10°范围立即结束"""

        target_yaw = self.normalize_angle(target_yaw)

        self.get_logger().info(
            f'开始普通转弯 -> 目标 {target_yaw:.1f}°'
        )

        while rclpy.ok():

            rclpy.spin_once(self, timeout_sec=0.01)

            current_yaw = self.get_current_yaw()

            if current_yaw is None:
                self.stop_robot()
                continue

            error = self.shortest_angle(
                target_yaw - current_yaw
            )

            abs_error = abs(error)

            if abs_error <= 10.0:
                self.stop_robot()
                self.get_logger().info(
                    f'✓ 普通转弯完成：'
                    f'当前={current_yaw:.2f}°，'
                    f'目标={target_yaw:.2f}°，'
                    f'误差={error:.2f}°'
                )
                return True

            if abs_error > 85.0:
                speed = 0.30
            elif abs_error > 70.0:
                speed = 0.25
            elif abs_error > 50.0:
                speed = 0.22
            elif abs_error > 35.0:
                speed = 0.18
            elif abs_error > 20.0:
                speed = 0.13
            elif abs_error > 12.0:
                speed = 0.10
            else:
                speed = 0.08

            cmd = Twist()

            if error > 0:
                cmd.angular.z = speed
            else:
                cmd.angular.z = -speed

            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)

        self.stop_robot()
        return False


    def rotate_light_yaw(self, target_yaw):
        """红绿灯专用朝向调整：进入±5°立即完成，不连续确认"""

        target_yaw = self.normalize_angle(target_yaw)

        self.get_logger().info(
            f'开始红绿灯朝向调整 -> 目标 {target_yaw:.1f}°'
        )

        while rclpy.ok():

            rclpy.spin_once(self, timeout_sec=0.01)

            current_yaw = self.get_current_yaw()

            if current_yaw is None:
                self.stop_robot()
                continue

            error = self.shortest_angle(
                target_yaw - current_yaw
            )

            abs_error = abs(error)

            # 红绿灯：进入±5°立即完成
            if abs_error <= 5.0:
                self.stop_robot()

                self.get_logger().info(
                    f'✓ 红绿灯朝向调整完成：'
                    f'当前={current_yaw:.2f}°，'
                    f'目标={target_yaw:.2f}°，'
                    f'误差={error:.2f}°'
                )
                return True

            # 红绿灯专用角速度
            if abs_error > 35.0:
                speed = 0.18
            elif abs_error > 20.0:
                speed = 0.14
            elif abs_error > 12.0:
                speed = 0.10
            elif abs_error > 8.0:
                speed = 0.08
            else:
                speed = 0.06

            cmd = Twist()

            if error > 0:
                cmd.angular.z = speed
            else:
                cmd.angular.z = -speed

            self.cmd_vel_pub.publish(cmd)

            time.sleep(0.05)

        self.stop_robot()
        return False


    def rotate_to_yaw(self, target_yaw):
        """旋转到 map 坐标系中的绝对目标角度"""

        target_yaw = self.normalize_angle(target_yaw)

        self.get_logger().info(
            f'开始精确角度校准 -> 目标 {target_yaw:.1f}°'
        )

        stable_count = 0

        while rclpy.ok():

            rclpy.spin_once(self, timeout_sec=0.01)

            current_yaw = self.get_current_yaw()

            if current_yaw is None:
                self.stop_robot()
                continue

            error = self.shortest_angle(
                target_yaw - current_yaw
            )

            abs_error = abs(error)

            # 连续2次进入3°范围，才认为真正校准完成
            if abs_error <= 3.0:

                self.stop_robot()
                stable_count += 1

                if stable_count >= 2:
                    self.get_logger().info(
                        f'✓ 精确校准完成：'
                        f'当前={current_yaw:.2f}°，'
                        f'目标={target_yaw:.2f}°，'
                        f'误差={error:.2f}°'
                    )
                    return True

                time.sleep(0.05)
                continue

            stable_count = 0

            # 精确校准专用角速度
            if abs_error > 85.0:
                speed = 0.25
            elif abs_error > 70.0:
                speed = 0.18
            elif abs_error > 50.0:
                speed = 0.14
            elif abs_error > 35.0:
                speed = 0.12
            elif abs_error > 20.0:
                speed = 0.10
            elif abs_error > 12.0:
                speed = 0.08
            elif abs_error > 8.0:
                speed = 0.06
            elif abs_error > 5.0:
                speed = 0.035
            else:
                speed = 0.025

            cmd = Twist()

            if error > 0:
                cmd.angular.z = speed
            else:
                cmd.angular.z = -speed

            self.cmd_vel_pub.publish(cmd)

            time.sleep(0.05)

        self.stop_robot()
        return False


    def rotate_relative(self, angle_deg):

        start_yaw = self.get_current_yaw()

        if start_yaw is None:
            self.get_logger().error(
                '无法获得当前角度，取消旋转'
            )
            return False

        target_yaw = self.normalize_angle(
            start_yaw + angle_deg
        )

        self.get_logger().info(
            f'开始旋转 {angle_deg:+.1f}° '
            f'→ 目标方向 {target_yaw:.1f}°'
        )

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.01
            )

            current_yaw = self.get_current_yaw()

            if current_yaw is None:
                continue

            error = self.shortest_angle(
                target_yaw - current_yaw
            )

            if abs(error) < 2.0:
                self.stop_robot()

                self.get_logger().info(
                    f'✓ 旋转完成，误差 {error:.2f}°'
                )
                return True

            angular_speed = (
                math.radians(abs(error)) * 1.5
            )

            angular_speed = min(
                angular_speed,
                0.22
            )

            angular_speed = max(
                angular_speed,
                0.035
            )

            cmd = Twist()

            if error > 0:
                cmd.angular.z = angular_speed
            else:
                cmd.angular.z = -angular_speed

            self.cmd_vel_pub.publish(cmd)

            time.sleep(0.05)

        return False

    # ==========================================================
    # RETURN_1 -> P0 最后一段专用直线返航
    # 不再交给Nav2，避免终点附近局部规划原地转圈
    # ==========================================================
    def direct_return_to_p0(self):

        # P0原始目标y=-1.522
        # 为避免车体越过终点撞南墙，提前约8cm停车
        safe_target_y = -1.44

        self.get_logger().info(
            '🏁 开始最后一段安全直线返航 P0'
        )

        # 最后一段只需要普通校准，不再使用慢速精确校准
        if not self.rotate_normal(-90.0):
            self.get_logger().error(
                '✗ 最终返航：-90°朝向校准失败'
            )
            self.stop_robot()
            return False

        self.get_logger().info(
            '✓ 最终方向已对准，开始低速直线返航'
        )

        start_time = time.time()

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.01
            )

            position = self.get_current_position()

            if position is None:
                self.stop_robot()
                time.sleep(0.05)
                continue

            x, y = position

            # 提前停车，不再等机器人中心到-1.522
            if y <= safe_target_y:
                self.stop_robot()

                self.get_logger().info(
                    f'✓ 安全到达P0终点区域：'
                    f'x={x:.3f}, y={y:.3f}'
                )

                # 不再进行第二次精确旋转
                self.get_logger().info(
                    '🏁 P0_RETURN：返航完成，机器人已停车'
                )
                self.get_logger().info(
                    '===================================='
                )
                self.get_logger().info(
                    '✓ 全部巡视任务完成'
                )
                self.get_logger().info(
                    '===================================='
                )

                return True

            if time.time() - start_time > 12.0:
                self.stop_robot()

                self.get_logger().error(
                    f'✗ 最终返航超时：'
                    f'x={x:.3f}, y={y:.3f}'
                )
                return False

            remaining = y - safe_target_y

            cmd = Twist()

            # 前面稍快
            if remaining > 0.20:
                cmd.linear.x = 0.07

            # 最后20cm主动减速
            elif remaining > 0.10:
                cmd.linear.x = 0.045

            # 最后10cm爬行，减少越界
            else:
                cmd.linear.x = 0.025

            # 最后一段禁止左右摆动
            cmd.angular.z = 0.0

            self.cmd_vel_pub.publish(cmd)

            time.sleep(0.05)

        self.stop_robot()
        return False

    # ==========================================================
    # TURN_3 -> PERSON_4 专用短距离直行
    # TURN_3 已经朝向180°，因此沿 -X 方向直接前进
    # ==========================================================
    def direct_turn3_to_person4(self):

        # PERSON_4正式坐标 x=0.010
        # 提前约5cm停车，避免越过观察位置
        safe_target_x = 0.060

        self.get_logger().info(
            '👤 TURN_3：开始直线前往PERSON_4观察位置'
        )

        # TURN_3已经完成180°校准，这里不再重复精确旋转
        self.stop_robot()

        start_time = time.time()

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.01
            )

            position = self.get_current_position()

            if position is None:
                self.stop_robot()
                time.sleep(0.05)
                continue

            x, y = position

            # 沿180°方向运动时x持续减小
            if x <= safe_target_x:
                self.stop_robot()

                self.get_logger().info(
                    f'✓ 已到达PERSON_4观察位置：'
                    f'x={x:.3f}, y={y:.3f}'
                )

                # 停稳
                end_time = time.time() + 0.3
                while time.time() < end_time and rclpy.ok():
                    rclpy.spin_once(
                        self,
                        timeout_sec=0.05
                    )

                return True

            # 安全超时
            if time.time() - start_time > 10.0:
                self.stop_robot()

                self.get_logger().error(
                    f'✗ TURN_3 -> PERSON_4直行超时：'
                    f'x={x:.3f}, y={y:.3f}'
                )
                return False

            remaining = x - safe_target_x

            cmd = Twist()

            # 前半段正常低速
            if remaining > 0.18:
                cmd.linear.x = 0.07

            # 接近PERSON_4后减速
            elif remaining > 0.08:
                cmd.linear.x = 0.045

            # 最后8cm爬行
            else:
                cmd.linear.x = 0.025

            # TURN_3已经朝180°，保持纯直行
            cmd.angular.z = 0.0

            self.cmd_vel_pub.publish(cmd)

            time.sleep(0.05)

        self.stop_robot()
        return False

    # ==========================================================
    # 当前导航点完成
    # ==========================================================
    def finish_current_point(self):

        name = self.waypoints[
            self.current_index
        ][0]

        self.get_logger().info(
            f'✓ {name} 动作完成'
        )

        self.current_index += 1

        end_time = time.time() + 0.5

        while time.time() < end_time and rclpy.ok():
            rclpy.spin_once(
                self,
                timeout_sec=0.05
            )

        self.send_next_goal()

    # ==========================================================
    # 停车
    # ==========================================================
    def stop_robot(self):

        msg = Twist()

        for _ in range(3):
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.03)

    @staticmethod
    def normalize_angle(angle):

        while angle > 180.0:
            angle -= 360.0

        while angle <= -180.0:
            angle += 360.0

        return angle

    @staticmethod
    def shortest_angle(angle):

        while angle > 180.0:
            angle -= 360.0

        while angle < -180.0:
            angle += 360.0

        return angle


def main(args=None):

    rclpy.init(args=args)

    node = PatrolNavigation()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok():
            node.stop_robot()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
