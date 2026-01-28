import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy

class JoyControl(Node):

    def __init__(self):
        super().__init__('ps_control_node')
        self.joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_listener_callback,
            10
        )
        self.joy_subscription

    def joy_listener_callback(self, msg):
        self.get_logger().info(f"Mensaje: {msg.data}")

def main():
    rclpy.init()
    node = JoyControl()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
