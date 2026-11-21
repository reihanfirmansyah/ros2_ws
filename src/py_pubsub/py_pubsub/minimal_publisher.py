import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Quaternion, 'quaternion_topic', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Quaternion()

        msg.x = 0.1
        msg.y = 0.2
        msg.z = 0.5
        msg.w = 1.0

        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Publishing Quaternion -> x: {msg.x}, y: {msg.y}, z: {msg.z}, w: {msg.w}'
        )


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()