#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPose(Node):

    def __init__(self):
        super().__init__('initial_pose')

        # 使用 Gazebo /clock 仿真时间
        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time',
                rclpy.Parameter.Type.BOOL,
                True
            )
        ])

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.amcl_wait_count = 0
        self.publish_count = 0
        self.amcl_detected = False

        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info(
            '等待 AMCL 启动，准备自动设置 P0...'
        )

    def timer_callback(self):

        node_names = self.get_node_names()

        # 等待 AMCL 节点出现
        if '/amcl' not in node_names and 'amcl' not in node_names:
            return

        # 第一次检测到 AMCL
        if not self.amcl_detected:
            self.amcl_detected = True
            self.get_logger().info(
                '检测到 AMCL，等待其完全进入工作状态...'
            )

        # 给 AMCL / lifecycle manager 足够的启动时间
        if self.amcl_wait_count < 8:
            self.amcl_wait_count += 1
            return

        msg = PoseWithCovarianceStamped()

        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        # P0 地图坐标
        msg.pose.pose.position.x = 1.645
        msg.pose.pose.position.y = -1.522
        msg.pose.pose.position.z = 0.0

        yaw = math.radians(90.542)

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # AMCL 初始协方差
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685

        self.publisher.publish(msg)

        self.publish_count += 1

        self.get_logger().info(
            '发送 P0 初始位姿 ({}/10)'.format(
                self.publish_count
            )
        )

        if self.publish_count >= 10:
            self.get_logger().info(
                'P0 自动初始定位完成。'
            )
            self.timer.cancel()


def main(args=None):

    rclpy.init(args=args)

    node = InitialPose()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
