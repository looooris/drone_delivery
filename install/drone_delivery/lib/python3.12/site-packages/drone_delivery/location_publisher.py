import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import SetBool


class DirectionPublisher(Node):
    def __init__(self, data):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Point, 'goto_robot', 1)
        timer_period = 1 # seconds
        self.data = data[0]
        self.announce = False
        if self.announce:
            self.create_timer(1, self.location_pub)
    
    def initalise_direciton(self, booleanDirection):
        self.announce = booleanDirection

    def location_pub(self):
        msg = Point()
        msg.x = float(self.data[0])
        msg.y = float(self.data[1])
        msg.z = float(self.data[2])
        self.publisher_.publish(msg)

class MetadataClient(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(SetBool, 'item_picker')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('item picker service not available, waiting again...')
        self.req = SetBool.Request()

    def send_request(self):
        return self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)
    data = [[2,2,0]]
    dir_pub = DirectionPublisher(data)
    robot_data = MetadataClient()
    future = robot_data.send_request()
    rclpy.spin_until_future_complete(robot_data, future)
    response = future.result()
    while response.data == False:
        print("robot not ready to takeoff")
    
    dir_pub.initalise_direciton(True)
    rclpy.spin(dir_pub)  
    

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dir_pub.destroy_node()
    robot_data.destory_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()