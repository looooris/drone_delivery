import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
import time
import math

from drone_delivery_services.srv import Destination
from drone_delivery_services.msg import Droneloc, Emergency, Goal
from drone_delivery import job_scheduling

class DirectionPublisher(Node):
    def __init__(self, data):
        super().__init__('direction_publisher')

        self.service = self.create_service(Destination, 'drone_destination_service', self.destination_callback)
        self.data = data
        self.robot_one_pos = None
        self.robot_two_pos = None
        self.emergency_history = False

        self.get_logger().info("Drone one goals: " + str(data[0]))
        self.get_logger().info("Drone two goals: " + str(data[1]))

        self.location_sub = self.create_subscription(Droneloc, 'drone_location', self.drone_location_callback, 10)
        self.emergency_stop = self.create_publisher(Emergency, 'drone_emergency', 3)
        self.goal_publisher = self.create_publisher(Goal, 'drone_goal', 3)
    
    def drone_location_callback(self, msg):
        if msg.id == 1:
            self.robot_one_pos = msg.currentposition
        else:
            self.robot_two_pos = msg.currentposition

        if self.robot_one_pos is not None and self.robot_two_pos is not None:
            distanceBetween = math.sqrt((self.robot_one_pos.x - self.robot_two_pos.x)**2 + (self.robot_one_pos.y - self.robot_two_pos.y) ** 2) ## only care about 2d to avoid robots landing ontop of each other
            #self.get_logger().info("Distance between robots: " + str(distanceBetween))
            emergency_message = Emergency()

            if distanceBetween < 2 and (abs(self.robot_one_pos.z - self.robot_two_pos.z) < 4) and ((self.robot_one_pos.x > 1 or self.robot_one_pos.x < -1) and (self.robot_one_pos.y > 1 or self.robot_one_pos.y < -1)):
                self.get_logger().info("Distance between drones is " + str(distanceBetween))

                emergency_message.id = 1
                emergency_message.safe = False
                self.emergency_history = True
                self.emergency_stop.publish(emergency_message)
            if distanceBetween > 1.75 and self.emergency_history:
                emergency_message.id = 1
                emergency_message.safe = True
                self.emergency_history = False
                self.emergency_stop.publish(emergency_message)

    async def destination_callback(self, request, response):
        # index the unique part of the drone id eg. 'drone_one'. 'one' is at index 'drone_one'[6:9]
        drone_idy = request.droneid[6:len(request.droneid)]

        # publishes first target location to drone 1 and second target location to drone 2
        # eg. data[0] goes to drone 1 and data[1] goes to drone 2
        if drone_idy == "one":
            dataToPeek = 0
        else:
            dataToPeek = 1
            
        # if the drone is close enough to the destination, it has reached the destination
        if abs(request.currentposition.x - float(self.data[dataToPeek][0][0])) < 0.5 and abs(request.currentposition.y - float(self.data[dataToPeek][0][1])) < 0.5:
            trip_duration = time.time() - request.starttime
            # drone has arrived at goal
            self.publish_goal("drone_"+drone_idy, self.data[dataToPeek][0], trip_duration)
            
            if drone_idy == "one":
                if len(self.data[0]) > 1: # if drone 1 has tasks left
                    # drone 1 completed task, so remove from task list
                    self.data[0].pop(0)
                else: # otherwise drone 1 is already home
                    response.deliverylocation.x = float(-1)
                    response.deliverylocation.y = float(-1)
                    response.deliverylocation.z = float(-1)
                    robot_one_pos = None
                    response.pharmacy = False
                    return response
            else:
                if len(self.data[1]) > 1: # if drone 2 has tasks left
                    # drone 2 completed task, so remove from task list
                    self.data[1].pop(0)
                else: # otherwise drone 2 is already home
                    response.deliverylocation.x = float(-1)
                    response.deliverylocation.y = float(-1)
                    response.deliverylocation.z = float(-1)
                    robot_two_pos = None
                    response.pharmacy = False
                    return response
                
        # drone is provided with new location to go to
        self.get_logger().info('Goal requested by ' + str(request.droneid))
        response.deliverylocation.x = float(self.data[dataToPeek][0][0])
        response.deliverylocation.y = float(self.data[dataToPeek][0][1])
        response.deliverylocation.z = float(self.data[dataToPeek][0][2])
        if self.data[dataToPeek][0][3] == 1: 
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
    data = job_scheduling.randomise_world(2)
    #data = [[[-45.65, 38.58, 0, 1], [54.04, 44.62, 0, 0], [-6.49895, -40.7371, 0, 1], [-60.92, 17.92, 0, 0], [-45.65, 38.58, 0, 1], [16.78, 40.43, 0, 0], [0, 0, 0, 1]], [[-6.49895, -40.7371, 0, 1], [-60.92, 17.92, 0, 0], [-45.65, 38.58, 0, 1], [54.04, 44.62, 0, 0], [1, 1, 0, 1]]]
    # ex. data = lists for each drone [ list for drone 1 [ 
    #                                      delivery location 1 [x, y, z, house(0) or pharmacy(1)],
    #                                       delivery location 2 [x, y, z, house(0) or pharmacy(1)],
    #                                       delivery location 3 [x, y, z, home base(1)],
    #                                    ], 
    #                                   list for drone 2 [ 
    #                                       delivery location 1 [x, y, z, house(0) or pharmacy(1)],
    #                                       delivery location 2 [x, y, z, house(0) or pharmacy(1)],
    #                                           delivery location 3 [x, y, z, house(0) or pharmacy(1)],
    #                                       delivery location 4 [x, y, z, home base(1)],
    #                                   ], 
    #                                 ]
    dir_pub = DirectionPublisher(data)
    
    rclpy.spin(dir_pub)

    dir_pub.destory_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()