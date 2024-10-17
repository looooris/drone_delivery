import rclpy
from geometry_msgs.msg import Twist

#HALF_DISTANCE_BETWEEN_WHEELS = 0.045
#WHEEL_RADIUS = 0.025

class DroneDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__camera = self.__robot.getDevice('camera')
        self.__gps = self.__robot.getDevice('gps')
        self.__gyro = self.__robot.getDevice('gyro') # Sets up fundamental objects

        # Propellers
        self.__propellers = [
            self.__robot.getDevice('front right propeller'),
            self.__robot.getDevice('front left propeller'),
            self.__robot.getDevice('rear right propeller'),
            self.__robot.getDevice('rear left propeller')
        ]
        for propeller in self.__propellers:
            propeller.setPosition(float('inf'))
            propeller.setVelocity(0)

        

        rclpy.init(args=None)
        self.__node = rclpy.create_node('control_'+self.__robot.name)
        self.__node.create_subscription(Twist, 'robot_control_' + self.__robot.name, self.__cmd_vel_callback, 1)
        #self.__node.create_subscription(None, 'cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        self.__propellers[0].setVelocity(-100)
        self.__propellers[1].setVelocity(100)
        self.__propellers[2].setVelocity(100)
        self.__propellers[3].setVelocity(-100)

        #command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        #command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        #self.__left_motor.setVelocity(command_motor_left)
        #self.__right_motor.setVelocity(command_motor_right)

def main():
    rclpy.init()
    robotControl = DroneDriver()
    rclpy.spin(robotControl)
    robotControl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()