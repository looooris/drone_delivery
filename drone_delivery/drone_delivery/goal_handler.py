import rclpy
from rclpy.node import Node
import time
from drone_delivery_services.msg import Goal
import csv


class GoalHandler(Node):
    def __init__(self):
        super().__init__('goal_handler')
        self.csv_file = 'drone_delivery/locations.csv'
        with open(self.csv_file, 'a', newline='') as csvfile:
            entries = csv.writer(csvfile)
            entries.writerow(['==='])

        # receives message when drone (or box?) is at goal, writes goal (and possibly delivery time) to csv
        self.receiver_sub = self.create_subscription(Goal, 'drone_goal', self.csv_callback, 10)


    def csv_callback(self, msg):
        # get goal data
        id = msg.id
        pos = msg.resultingposition
        goaltype = 'H' if  msg.goaltype == 0 else 'P'
        deliverytime = msg.deliverytime
        # record in csv
        
        self.write_goal(id, pos, goaltype, deliverytime)

    def write_goal(self, id, pos, goaltype, deliverytime):
        # write goal to csv
        with open(self.csv_file, 'a', newline='') as csvfile:
            self.get_logger().info("writing to csv")
            entries = csv.writer(csvfile)
            # id,x,y,z,goaltype,deliverytyime
            # ex. drone_one,1,1,1,P,26.72
            entries.writerow([id, pos.x, pos.y, pos.z, goaltype, abs(deliverytime)])


def main():
    rclpy.init()
    goal_handler = GoalHandler()
    rclpy.spin(goal_handler)
    goal_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

