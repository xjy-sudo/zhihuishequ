import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


class BuildingTemperatureController(Node):

    def __init__(self):
        super().__init__('building_temperature_controller')

        self.temperatures = {
            'building_a': 25.0,
            'building_b': 25.0,
            'building_c': 25.0,
            'building_d': 25.0,
        }

        self.create_subscription(
            Float32,
            '/building_a/temperature',
            lambda msg: self.temperature_callback('building_a', msg),
            10
        )

        self.create_subscription(
            Float32,
            '/building_b/temperature',
            lambda msg: self.temperature_callback('building_b', msg),
            10
        )

        self.create_subscription(
            Float32,
            '/building_c/temperature',
            lambda msg: self.temperature_callback('building_c', msg),
            10
        )

        self.create_subscription(
            Float32,
            '/building_d/temperature',
            lambda msg: self.temperature_callback('building_d', msg),
            10
        )

        self.get_logger().info(
            'Building temperature controller started'
        )

        self.print_all_temperatures()

    def temperature_callback(self, building, msg):

        temperature = float(msg.data)

        self.temperatures[building] = temperature

        if temperature >= 50.0:
            state = 'HIGH TEMPERATURE'
        else:
            state = 'NORMAL'

        self.get_logger().info(
            '{} -> {:.1f} C [{}]'.format(
                building,
                temperature,
                state
            )
        )

    def print_all_temperatures(self):

        self.get_logger().info(
            'Initial temperatures: '
            'A={:.1f} C, B={:.1f} C, '
            'C={:.1f} C, D={:.1f} C'.format(
                self.temperatures['building_a'],
                self.temperatures['building_b'],
                self.temperatures['building_c'],
                self.temperatures['building_d']
            )
        )


def main(args=None):

    rclpy.init(args=args)

    node = BuildingTemperatureController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
