import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node

from drone_delivery_services.srv import Gripper

class GripperControl(Node):
    def __init__(self):
        super().__init__('gripper_service')
        self.gripperOpen = True # used to measure if gripper is open
        self.service = self.create_service(Gripper, 'drone_two_gripper', self.gripper_callback)

    def gripper_callback(self, request, response):
        self.get_logger().info('Gripper requested') # outputs a message in terminal to alert the user

        if request.open:
            # attempts to open gripper
            if self.gripperOpen:
                # returns error message if gripper is already open
                response.success = False
                self.get_logger().info('Gripper open requested but already open')
                return response
            else:
                # opens gripper
                self.gripperOpen = True
                response.success = True
                self.get_logger().info('Gripper open requested')
                return response
        else:
            # attempts to close gripper
            if not self.gripperOpen:
                # returns error message if gripper is already closed
                response.success = False
                self.get_logger().info('Gripper close requested but already closed')
                return response
            else:
                # closes gripper
                self.gripperOpen = False
                response.success = True
                self.get_logger().info('Gripper close requested')
                return response

def main(args=None):
    rclpy.init(args=args)
    grip_ctrl = GripperControl()
    
    rclpy.spin(grip_ctrl)

    robot_data.destory_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()