import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
import time

from drone_delivery_services.srv import Destination

class DirectionPublisher(Node):
    def __init__(self, data):
        super().__init__('minimal_service')

        self.service = self.create_service(Destination, 'drone_destination_service', self.destination_callback)
        self.data = data

    async def destination_callback(self, request, response):
        drone_idy = request.droneid[6:len(request.droneid)]
        if drone_idy == "one":
            dataToPeek = 0
        else:
            dataToPeek = 1
        if abs(request.currentposition.x - float(self.data[dataToPeek][0][0])) < 0.5 and abs(request.currentposition.y - float(self.data[dataToPeek][0][1])) < 0.5:
            if drone_idy == "one":
                if len(self.data[0]) > 1:
                    self.data[0].pop(0)
                else:
                    #self.get_logger().info('Robot finished')
                    response.deliverylocation.x = float(0)
                    response.deliverylocation.y = float(0)
                    response.deliverylocation.z = float(0)
                    response.pharmacy = False
                    return response
            else:
                if len(self.data[1]) > 1:
                    self.data[1].pop(0)
                else:
                    #self.get_logger().info('Robot finished')
                    response.deliverylocation.x = float(0)
                    response.deliverylocation.y = float(0)
                    response.deliverylocation.z = float(0)
                    response.pharmacy = False
                    return response
               
            
        
        # else:
        #     self.get_logger().info('Requested when not at target')
        self.get_logger().info('Goal requested by ' + str(request.droneid))
        response.deliverylocation.x = float(self.data[dataToPeek][0][0])
        response.deliverylocation.y = float(self.data[dataToPeek][0][1])
        response.deliverylocation.z = float(self.data[dataToPeek][0][2])
        if self.data[dataToPeek][0][3] == 1: 
            response.pharmacy = True 
        else: 
            response.pharmacy = False
        return response
    
def main(args=None):
    rclpy.init(args=args)

    # do path calculations
    
    data = [[[-5, -5, 0, 1], [0, 0, 0, 0]], [[5, 5, 0, 1], [1, 1, 0, 0]]]
    dir_pub = DirectionPublisher(data)
    
    rclpy.spin(dir_pub)

    dir_pub.destory_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()