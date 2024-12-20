import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
import time

from drone_delivery_services.srv import Destination
from drone_delivery_services.msg import Goal
from drone_delivery import job_scheduling


class DirectionPublisher(Node):
    def __init__(self, data):
        super().__init__('direction_publisher')

        self.service = self.create_service(Destination, 'drone_destination_service', self.destination_callback)
        self.data = data
        self.get_logger().info("Drone one goals: " + str(data))

        self.goal_publisher = self.create_publisher(Goal, 'drone_goal', 3)

    async def destination_callback(self, request, response):
        self.get_logger().info('Goal requested')

        if abs(request.currentposition.x - float(self.data[0][0])) < 0.5 and abs(request.currentposition.y - float(self.data[0][1])) < 0.5:
            trip_duration = time.time() - request.starttime
            self.publish_goal("drone_one", self.data[0], trip_duration)

            if len(self.data) > 1:
                self.data.pop(0)

            else:
                #self.get_logger().info('Robot finished')
                response.deliverylocation.x = float(-1)
                response.deliverylocation.y = float(-1)
                response.deliverylocation.z = float(-1)
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

    def publish_goal(self, droneid, goal_data, trip_duration):
        goal_message = Goal()
        # [x, y, z, landing location type (0 or 1)]
        x, y, z, goaltype = goal_data
        goal_message.id = droneid
        goal_message.resultingposition.x = float(x)
        goal_message.resultingposition.y = float(y)
        goal_message.resultingposition.z = float(z)
        goal_message.goaltype = int(goaltype)
        goal_message.deliverytime = float(trip_duration)

        self.goal_publisher.publish(goal_message)
    
def main(args=None):
    rclpy.init(args=args)

    # do path calculations
    
    data = job_scheduling.randomise_world(1)
    #data=[[[-45.65, 38.58, 0, 0],[-60.65, -27.63, 0, 0],[0,0,0,0]]]
    dir_pub = DirectionPublisher(data[0])
    
    rclpy.spin(dir_pub)

    dir_pub.destory_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()