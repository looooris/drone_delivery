import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node

from drone_delivery_services.srv import Gripper



class GripperControl(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.gripperOpen = True
        self.service = self.create_service(Gripper, 'drone_gripper', self.gripper_callback)

    def gripper_callback(self, request, response):
        self.get_logger().info('Gripper requested')

        if request.open:
            if self.gripperOpen:
                response.success = False
                self.get_logger().info('Gripper open requested but already open')
                return response
            else:
                self.gripperOpen = True
                response.success = True
                self.get_logger().info('Gripper open requested')
                return response
        else:
            if not self.gripperOpen:
                response.success = False
                self.get_logger().info('Gripper close requested but already closed')
                return response
            else:
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