import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
import time

from drone_delivery_services.srv import Destination



class DirectionPublisher(Node):
    def __init__(self, data):
        super().__init__('minimal_service')
        self.service = self.create_service(Destination, 'drone_destination', self.destination_callback)
        # self.publisher_ = self.create_publisher(Point, 'goto_robot', 1)
        # timer_period = 10 # seconds
        self.data = data
        # #self.announce = False
        # self.create_timer(1, self.location_pub)

    async def destination_callback(self, request, response):
        self.get_logger().info('Goal requested')
        #time.sleep(5)

        if abs(request.currentposition.x - float(self.data[0][0])) < 0.5 and abs(request.currentposition.y - float(self.data[0][1])) < 0.5:
            if len(self.data) > 1:
                self.data.pop(0)

            else:
                self.get_logger().info('Robot finished')
                raise Exception("Robot is finished.")
                return response
        
        else:
            self.get_logger().info('Requested when not at target')
        
        response.deliverylocation.x = float(self.data[0][0])
        response.deliverylocation.y = float(self.data[0][1])
        response.deliverylocation.z = float(self.data[0][2])
        if self.data[0][3] == 1: 
            response.pharmacy = True 
        else: 
            response.pharmacy = False
        return response
    


def main(args=None):
    rclpy.init(args=args)

    # do path calculations
    #     
    data = [[-60, 15, 0, 1], [0, 0, 0, 0]]
    dir_pub = DirectionPublisher(data)
    
    rclpy.spin(dir_pub)

    dir_pub.destory_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()