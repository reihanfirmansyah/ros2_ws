import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Quaternion,
            'quaternion_topic',
            self.listener_callback,
            10
        )
        self.subscription

    def quaternion_to_rpy(self, x: float, y: float, z: float, w: float):
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm == 0.0:
            return 0.0, 0.0, 0.0
        x, y, z, w = x / norm, y / norm, z / norm, w / norm

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    def get_robot_response(self, roll: float, pitch: float, yaw: float) -> str:
        if abs(roll) > 20.0 or abs(pitch) > 20.0:
            return 'DIAM / STOP DARURAT (Badan miring melebihi batas aman, butuh stabilisasi)'
        elif yaw > 15.0:
            return 'PUTAR BADAN KE KIRI (Menyesuaikan arah hadap, yaw positif)'
        elif yaw < -15.0:
            return 'PUTAR BADAN KE KANAN (Menyesuaikan arah hadap, yaw negatif)'
        else:
            return 'JALAN MAJU (Posisi stabil dan arah hadap lurus)'

    def listener_callback(self, msg: Quaternion):
        roll, pitch, yaw = self.quaternion_to_rpy(msg.x, msg.y, msg.z, msg.w)
        response = self.get_robot_response(roll, pitch, yaw)

        self.get_logger().info(
            f'RPY (deg) -> Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}° | '
            f'Respon Robot: {response}'
        )


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()