import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
import time
import math

from drone_delivery_services.srv import Destination
from drone_delivery_services.msg import Droneloc, Emergency


class DirectionPublisher(Node):
    def __init__(self, data):
        super().__init__('direction_publisher')

        self.data = data
        #self.get_logger().info(str(data))
        self.robot_one_pos = None
        self.robot_two_pos = None

        self.service = self.create_service(Destination, 'drone_destination_service', self.destination_callback)
        self.location_sub = self.create_subscription(Droneloc, 'drone_location', self.drone_location_callback, 10)
        self.emergency_stop = self.create_publisher(Emergency, 'drone_emergency', 3)
        
        

    def drone_location_callback(self, msg):
        if msg.id == 1:
            self.robot_one_pos = msg.currentposition
        else:
            self.robot_two_pos = msg.currentposition

        if self.robot_one_pos is not None and self.robot_two_pos is not None:
            distanceBetween = math.sqrt((self.robot_one_pos.x - self.robot_two_pos.x)**2 + (self.robot_one_pos.y - self.robot_two_pos.y) ** 2 + (self.robot_one_pos.z - self.robot_two_pos.z)**2)
            self.get_logger().info(str(distanceBetween))
            emergency_message = Emergency()

            while distanceBetween < 0.5:
                self.get_logger().info("Distance between drones is " + str(distanceBetween))

                # check if a delivery is more urgent than another
                
                emergency_message.id = 1
                emergency_message.safe = False
                self.emergency_stop.publish(emergency_message)
                distanceBetween = math.sqrt((self.robot_one_pos.x - self.robot_two_pos.x)**2 + (self.robot_one_pos.y - self.robot_two_pos.y) ** 2 + (self.robot_one_pos.z - self.robot_two_pos.z)**2)

            if not emergency_message.safe:
                emergency_message = Emergency()
                #same as above
                emergency_message.id = 1
                emergency_message.safe = True
                self.emergency_stop.publish(emergency_message)

            
    
    async def destination_callback(self, request, response):
        # index the unique part of the drone id eg. 'drone_one'. 'one' is at index 'drone_one'[6:9]
        drone_idy = request.droneid[6:len(request.droneid)]
        response = Destination.Response()

        # publishes first target location to drone 1 and second target location to drone 2
        # eg. data[0] goes to drone 1 and data[1] goes to drone 2
        if drone_idy == "one":
            dataToPeek = 0
        else:
            dataToPeek = 1
            
        # if the drone is at or close enough to the ground, it has reached the delivery location
        if abs(request.currentposition.x - float(self.data[dataToPeek][0][0])) < 0.5 and abs(request.currentposition.y - float(self.data[dataToPeek][0][1])) < 0.5:
            if drone_idy == "one":
                if len(self.data[0]) > 1:
                    self.data[0].pop(0)
                else:
                    self.get_logger().info('Robot one finished')
                    response.deliverylocation.x = float(0)
                    response.deliverylocation.y = float(0)
                    response.deliverylocation.z = float(0)
                    response.pharmacy = False
                    response.finished = True
                    return response
            else:
                if len(self.data[1]) > 1:
                    self.data[1].pop(0)
                else:
                    self.get_logger().info('Robot two finished')
                    response.deliverylocation.x = float(1)
                    response.deliverylocation.y = float(1)
                    response.deliverylocation.z = float(0)
                    response.pharmacy = False
                    response.finished = True
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
        #self.get_logger().info('Sent ' + str(response) +' ' + str(request.droneid))
        return response
    
def main(args=None):
    rclpy.init(args=args)

    # do path calculations
    
    data = [[[5, 5, 0, 1], [0, 0, 0, 0]], [[-5, -5, 0, 1], [1, 1, 0, 0]]]
    dir_pub = DirectionPublisher(data)
    
    rclpy.spin(dir_pub)

    dir_pub.destory_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()