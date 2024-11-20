import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
import time

from drone_delivery_services.srv import Destination

class DirectionPublisher(Node):
    def __init__(self, data):
        super().__init__('minimal_service')

        self.service = self.create_service(Destination, 'drone_one_destination', self.destination_callback)
        self.data = data

    async def destination_callback(self, request, response):
        self.get_logger().info('Goal requested')

        if abs(request.currentposition.x - float(self.data[0][0])) < 0.5 and abs(request.currentposition.y - float(self.data[0][1])) < 0.5:
            # if self.data[0][3] == 1: # goal was pharmacy, send request
            #     self.movementbot_target = request.currentposition
            #    # while not self.wheelbotDest:
            #     #    pass
            #     #    #self.get_logger().info("Wheelbot is not at dest, cannot continue")
            #     #self.movementbot_target = None
            if len(self.data) > 1:
                self.data.pop(0)

            else:
                #self.get_logger().info('Robot finished')
                response.deliverylocation.x = float(0)
                response.deliverylocation.y = float(0)
                response.deliverylocation.z = float(0)
                response.pharmacy = False
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
    
    data = [[5, 5, 0, 1], [0, 0, 0, 0]]
    dir_pub = DirectionPublisher(data)
    
    rclpy.spin(dir_pub)

    dir_pub.destory_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()