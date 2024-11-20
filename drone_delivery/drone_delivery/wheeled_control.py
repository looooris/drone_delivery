import rclpy
from rclpy.action import ActionServer
from geometry_msgs.msg import Point
from controller import Robot
import math
import time

from drone_delivery_services.msg import Boolean as ROSBool

class PharmDriver():
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
       
        self.time_step = 1

        # Initalise sensors
        self.computer = self.robot.getDevice('inertial unit')

        self.gps = self.robot.getDevice('gps')

        # Enable sensors
        self.computer.enable(self.time_step)
        self.gps.enable(self.time_step)

        # Initalise motors
        self.lm = self.robot.getDevice('left wheel motor')
        self.rm = self.robot.getDevice('right wheel motor')

        self.lm.setPosition(float('inf'))
        self.lm.setVelocity(0)

        self.rm.setPosition(float('inf'))
        self.rm.setVelocity(0)

        # Robot metadata
        self.at_target = False

        # Destination metadata
        self.home_location = Point()
        initial = self.gps.getValues()
        self.home_location.x = initial[0]
        self.home_location.y = initial[1]
        self.home_location.z = initial[2]
        self.robot_target = Point()
        self.robot_target.x = 0
        self.robot_target.y = 0.2
        self.robot_target.z = 0

        # Configure ROS interface
        rclpy.init(args=None)
        self.subscription = rclpy.create_node('driver_wheeled') 
        self.destinationSubscription = self.subscription.create_subscription(Point, 'wheel_destination', self.wheel_destcallback, 10)

        self.movementbot_client = self.subscription.create_publisher(ROSBool, 'wheel_destination_feedback', 1)
        self.movementbot_timer = self.subscription.create_timer(1, self.movementbot_callback)
        self.moveConfirm = ROSBool()

    def movementbot_callback(self):
        if self.at_target:
            self.moveRequest = True  
            self.movementbot_client.publish(self.moveRequest)
            self.robot_target = self.home_location

    def wheel_destcallback(self, msg):
        pass
        # self.subscription.get_logger().info("Destination received" + str(msg))
        # self.robot_target.x = msg.x
        # self.robot_target.y = msg.y + 0.2
        # self.robot_target.z = msg.z

    def calcAngleDist(self, gps, imu):
        diff_x = self.robot_target.x - gps[0] 
        diff_y = self.robot_target.y - gps[1]    
        distance = math.sqrt((diff_x)**2 + (diff_y)**2)

        angleToMove = math.atan2((diff_y),(diff_x)) - imu[2] 

        if (angleToMove >  math.pi):
            angleToMove -= 2 * math.pi

        return distance, angleToMove

    def step(self): 
        rclpy.spin_once(self.subscription, timeout_sec=0)
        gpsValues = self.gps.getValues()
        imuVaues = self.computer.getRollPitchYaw()
        self.subscription.get_logger().info(str(imuVaues))
    
        if abs(self.robot_target.x - gpsValues[0]) > 0.05 or abs(self.robot_target.y - gpsValues[1]) > 0.05:
            distance, angle = self.calcAngleDist(gpsValues, imuVaues)

            if distance > 0:
                left_motor = 5
                right_motor = 5
            else:
                left_motor = 5
                right_motor = 5

            if abs(angle) > 0.1:
                left_motor -= 2
                right_motor += 2

            #left_motor = (distance - angle * 0.045) / 0.025
            #right_motor = (distance + angle * 0.045) / 0.025
            
            self.lm.setVelocity(left_motor)
            self.rm.setVelocity(right_motor)

            #self.subscription.get_logger().info(str(distance) + str(angle))
            #self.subscription.get_logger().info(str(self.robot_target.x))

        #self.__target_twist = Twist()
        

def main():
    wheelControl = PharmDriver()
    rclpy.spin(wheelControl)

    wheelControl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()