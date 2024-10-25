import rclpy
from geometry_msgs.msg import Twist
from controller import Robot

#HALF_DISTANCE_BETWEEN_WHEELS = 0.045
#WHEEL_RADIUS = 0.025
    
def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)

class DroneDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        self.time_step = 1
        self.name = "drone_one"

        # Initalise and enable sensors
        self.camera = self.robot.getDevice('camera')
        self.gps = self.robot.getDevice('gps')
        self.computer = self.robot.getDevice('inertial unit')
        self.gyro = self.robot.getDevice('gyro') 

        self.camera.enable(self.time_step)
        self.gps.enable(self.time_step)
        self.gyro.enable(self.time_step)
        self.computer.enable(self.time_step)

        # Initalise propellers
        self.frp = self.robot.getDevice('front right propeller')
        self.flp = self.robot.getDevice('front left propeller')
        self.rrp = self.robot.getDevice('rear right propeller')
        self.rlp = self.robot.getDevice('rear left propeller')
        self.__propellerList = [self.frp, self.flp, self.rrp, self.rlp]

        for propeller in self.__propellerList:
            propeller.setPosition(float('inf'))
            propeller.setVelocity(0)
        self.target_altitude = 3

        # Configure ROS interface
        rclpy.init()
        self.subscription = rclpy.create_node('driver_'+self.name) 
        self.subscription.create_subscription(Twist, 'cmd_vel_' + self.name, self.__cmd_vel_callback, 1)
        #self.__node.create_subscription(None, 'cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def locateDrone(self):
        internalComputerValues = self.computer.getRollPitchYaw()
        gpsValues = self.gps.getValues()
        droneVelocity = self.gps.getSpeed()
        gyroValues = rollVelocity, pitchVelocity, yawVelocity = self.gyro.getValues()
        return internalComputerValues, gpsValues, gyroValues, droneVelocity

    def bindValue(self, value):
        return min(max(value, -1), 1)

    def step(self):
        rclpy.spin_once(self.subscription, timeout_sec=0)
        self.__target_twist = Twist()

        #Where is the drone?
        intComVal, gpsVal, gyroVal, droneVelocity = self.locateDrone()
        
        print("Roll, Pitch, Yaw: " + str(intComVal[0]) + " " + str(intComVal[1]) + " " + str(intComVal[2]))
        print("PosX, PosY, PosZ" + str(gpsVal[0]) + " " + str(gpsVal[1]) + " " + str(gpsVal[2]))  

        roll_input = 50 * clamp(intComVal[0], -1, 1) + gyroVal[0]
        pitch_input = 30.0 * clamp(intComVal[1], -1, 1) + gyroVal[1]
        yaw_input = 2 * (self.__target_twist.angular.z - gyroVal[2])
        clamped_difference_altitude = clamp(self.target_altitude - gpsVal[2] + 0.6, -1, 1)
        vertical_input = 3.0 * clamped_difference_altitude**3.0

        front_left_motor_input = 70 + vertical_input - yaw_input + pitch_input - roll_input
        front_right_motor_input = 70 + vertical_input + yaw_input + pitch_input + roll_input
        rear_left_motor_input = 70 + vertical_input + yaw_input - pitch_input - roll_input
        rear_right_motor_input = 70 + vertical_input - yaw_input - pitch_input + roll_input
        self.frp.setVelocity(-front_right_motor_input) # Front Right
        self.flp.setVelocity(front_left_motor_input) # Front Left
        self.rrp.setVelocity(rear_right_motor_input) # Rear Right
        self.rlp.setVelocity(-rear_left_motor_input) # Rear Left

        self.__propellerList[0].setVelocity(-100) # Front Right
        self.__propellerList[1].setVelocity(100) # Front Left
        self.__propellerList[2].setVelocity(100) # Rear Right
        self.__propellerList[3].setVelocity(-100) # Rear Left

def main():
    rclpy.init()
    robotControl = DroneDriver()
    rclpy.spin(robotControl)
    #robotControl.moveRobot()
    robotControl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()