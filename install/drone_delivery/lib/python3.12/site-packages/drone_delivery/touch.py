import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from random import randint

class RobotCompare(Node):
    def __init__(self):
        self._data = None
        super().__init__('subscriber')
        self.subscription = self.create_subscription(
            Point,
            '/drone_one/gps',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
    def listener_callback(self, msg):
        self._data = msg
        self.get_logger().info('Data Updated')
        self.get_logger().info('X: "%s"' % msg.x)
        self.get_logger().info('Y: "%s"' % msg.y)
        self.get_logger().info('Z: "%s"' % msg.z)

    def data(self):
        return self._data


def main(args=None):
    rclpy.init(args=args)
    robotOne = RobotCompare()
    rclpy.spin_once(robotOne)
    robotOne.destroy_node()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #robotOne.destroy_node()
    #robotTwo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
