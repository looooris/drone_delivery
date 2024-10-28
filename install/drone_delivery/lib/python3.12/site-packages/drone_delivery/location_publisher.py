import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    def __init__(self, data):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Point, 'goto_robot', 1)
        timer_period = 1 # seconds
        self.data = data
        self.create_timer(timer_period, self.location_pub)

    def location_pub(self):
        if len(self.data) != 0:
            self.instruction = self.data.pop()
        msg = Point()
        msg.x = float(self.instruction[0])
        msg.y = float(self.instruction[1])
        msg.z = float(self.instruction[2])
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    data = [[2,2,0]]
    minimal_publisher = MinimalPublisher(data)
    rclpy.spin(minimal_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()