import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from std_msgs.msg import String
from webots_ros2_msgs.srv import GetBool



class DirectionPublisher(Node):
    def __init__(self, data):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Point, 'goto_robot', 1)
        timer_period = 10 # seconds
        self.data = data
        #self.announce = False
        self.create_timer(1, self.location_pub)
    
    def update_location(self, data):
        self.data = data

    def location_pub(self):
        msg = Point()
        msg.x = float(self.data[0])
        msg.y = float(self.data[1])
        msg.z = float(self.data[2])
        self.publisher_.publish(msg)


class MetadataClient(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription  = self.create_subscription(Point, 'robot_metadata', self.metadata_organiser, 1)
        self.launchable = None
        self.atTarget = None
        self.itemGrabbed = None

    def return_metadata(self):
        print([self.launchable, self.itemGrabbed, self.atTarget])
        return [self.launchable, self.itemGrabbed, self.atTarget]

    def metadata_organiser(self, msg):
        if msg.x > 0.5:
            self.launchable = True
        else:
            self.launchable = False
        if msg.y > 0.5:
            self.itemGrabbed = True
        else:
            self.itemGrabbed = False
        if msg.z > 0.5:
            self.atTarget = True
        else:
            self.atTarget = False


def main(args=None):
    rclpy.init(args=args)
    data = [[0,0,0]]
    dir_pub = DirectionPublisher(data[0])
    robot_data = MetadataClient()
    while robot_data.return_metadata()[0] == False:
         print("robot not ready to takeoff")

    print(robot_data.return_metadata())
    rclpy.spin(dir_pub)  
    

    # # Destroy the node explicitly
    # # (optional - otherwise it will be done automatically
    # # when the garbage collector destroys the node object)
    # dir_pub.destroy_node()
    # robot_data.destory_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()