import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
import time

from drone_delivery_services.srv import Destination

class DirectionPublisher(Node):
    def __init__(self, dataOne, dataTwo):
        super().__init__('minimal_service')

        self.service = self.create_service(Destination, 'drone_one_destination', self.destination_callback)
        self.service = self.create_service(Destination, 'drone_two_destination', self.destinationTwo_callback)
        self.dataOne = dataOne
        self.dataTwo = dataTwo

    async def destination_callback(self, request, response):
        

        if abs(request.currentposition.x - float(self.dataOne[0][0])) < 0.5 and abs(request.currentposition.y - float(self.dataOne[0][1])) < 0.5:
            # if self.data[0][3] == 1: # goal was pharmacy, send request
            #     self.movementbot_target = request.currentposition
            #    # while not self.wheelbotDest:
            #     #    pass
            #     #    #self.get_logger().info("Wheelbot is not at dest, cannot continue")
            #     #self.movementbot_target = None
            if len(self.dataOne) > 1:
                self.dataOne.pop(0)

            else:
                #self.get_logger().info('Robot finished')
                response.deliverylocation.x = float(0)
                response.deliverylocation.y = float(0)
                response.deliverylocation.z = float(0)
                response.pharmacy = False
                return response
        
        # else:
        #     self.get_logger().info('Requested when not at target')
        self.get_logger().info('Goal one requested')
        response.deliverylocation.x = float(self.dataOne[0][0])
        response.deliverylocation.y = float(self.dataOne[0][1])
        response.deliverylocation.z = float(self.dataOne[0][2])
        if self.dataOne[0][3] == 1: 
            response.pharmacy = True 
        else: 
            response.pharmacy = False
        return response

    
    async def destinationTwo_callback(self, request, response):
        self.get_logger().info('Goal requested')

        if abs(request.currentposition.x - float(self.dataTwo[0][0])) < 0.5 and abs(request.currentposition.y - float(self.dataTwo[0][1])) < 0.5:
            # if self.data[0][3] == 1: # goal was pharmacy, send request
            #     self.movementbot_target = request.currentposition
            #    # while not self.wheelbotDest:
            #     #    pass
            #     #    #self.get_logger().info("Wheelbot is not at dest, cannot continue")
            #     #self.movementbot_target = None
            time.wait(2)
            if len(self.dataTwo) > 1:
                self.dataTwo.pop(0)

            else:
                #self.get_logger().info('Robot finished')
                response.deliverylocation.x = float(0)
                response.deliverylocation.y = float(0)
                response.deliverylocation.z = float(0)
                response.pharmacy = False
                return response
        
        # else:
        #     self.get_logger().info('Requested when not at target')
        
        self.get_logger().info('Goal two requested')
        response.deliverylocation.x = float(self.dataTwo[0][0])
        response.deliverylocation.y = float(self.dataTwo[0][1])
        response.deliverylocation.z = float(self.dataTwo[0][2])
        if self.dataTwo[0][3] == 1: 
            response.pharmacy = True 
        else: 
            response.pharmacy = False
        return response
    
def main(args=None):
    rclpy.init(args=args)

    # do path calculations
    
    data = [[-5, -5, 0, 1], [0, 0, 0, 0]]
    twata = [[5, 5, 0, 1], [1, 1, 0, 0]]
    dir_pub = DirectionPublisher(data, twata)
    
    rclpy.spin(dir_pub)

    dir_pub.destory_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()