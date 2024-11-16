import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node

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

    def destination_callback(self, request, response):
        self.get_logger().info('Goal requested')

        if abs(request.currentposition.x - float(self.data[0][0])) < 0.5 and abs(request.currentposition.y - float(self.data[0][1])) < 0.5:
            if len(self.data) > 0:
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
    #data = [[-60.65, -27.63, 0, 0]]
    #data = [[-45.65, 38.58, 0, 1],[30.55, -26.11, 0, 0], [0, 0, 0, 0]]
    data = [[5, 5, 0, 1], [-5, -5, 0, 0], [0, 0, 0, 0]]
    dir_pub = DirectionPublisher(data)

    #dir_pub.send_goal()
    
    rclpy.spin(dir_pub)
    # robot_data = MetadataClient()
    # while robot_data.return_metadata()[0] == False:
    #      print("robot not ready to takeoff")


    # print(robot_data.return_metadata())
    #
    # rclpy.spin(dir_pub)  
    

    # # Destroy the node explicitly
    # # (optional - otherwise it will be done automatically
    # # when the garbage collector destroys the node object)
    # dir_pub.destroy_node()
    # robot_data.destory_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()