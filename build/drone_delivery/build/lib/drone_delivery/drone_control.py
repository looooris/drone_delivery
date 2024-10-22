import rclpy
from geometry_msgs.msg import Twist
from controller import Robot

#HALF_DISTANCE_BETWEEN_WHEELS = 0.045
#WHEEL_RADIUS = 0.025
TAKEOFF_VELOCITY = 75

def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)

class DroneDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        self.time_step = 8
        self.name = "drone1"

        # Initalise and enable sensors
        self.camera = self.robot.getDevice('camera')
        self.gps = self.robot.getDevice('gps')
        self.computer = self.robot.getDevice('inertial unit')
        self.gyro = self.robot.getDevice('gyro') 

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

        self.current_pose = 6 * [0]  # X, Y, Z, yaw, pitch, roll
        self.target_position = [0, 0, 0]
        self.target_index = 0
        self.target_altitude = 0

        # Configure ROS interface
        rclpy.init()
        self.subscription = rclpy.create_node('driver_'+self.name) 
        self.subscription.create_subscription(Twist, 'cmd_vel_' + self.name, self.__cmd_vel_callback, 1)
        #self.__node.create_subscription(None, 'cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    # def step(self):
    #     rclpy.spin_once(self.subscription, timeout_sec=0)

    #     self.target_altitude = 25

    #     #Where is the drone?
    #     droneRoll, dronePitch, droneYaw = self.computer.getRollPitchYaw()
    #     positionX, positionY, vertical = self.gps.getValues()
    #     droneVelocity = self.gps.getSpeed()
    #     rollVelocity, pitchVelocity, yawVelocity = self.gyro.getValues()
    #     print("Roll, Pitch, Yaw: " + str(droneRoll) + str(dronePitch) + str(droneYaw))
    #     print("PosX, PosY, Vertical" + str(positionX) + str(positionY) + str(vertical))        

    #     while vertical < 25:
    #         roll_input = 50 * clamp(droneRoll, -1, 1) + rollVelocity
    #         pitch_input = 30.0 * clamp(dronePitch, -1, 1) + pitchVelocity
    #         yaw_input = 0
    #         clamped_difference_altitude = clamp(self.target_altitude - vertical + 0.6, -1, 1)
    #         vertical_input = 3.0 * pow(clamped_difference_altitude, 3.0)

    #         front_left_motor_input = 68.5 + vertical_input - yaw_input + pitch_input - roll_input
    #         front_right_motor_input = 68.5 + vertical_input + yaw_input + pitch_input + roll_input
    #         rear_left_motor_input = 68.5 + vertical_input + yaw_input - pitch_input - roll_input
    #         rear_right_motor_input = 68.5 + vertical_input - yaw_input - pitch_input + roll_input
    #         self.__propellerList[0].setVelocity(-front_right_motor_input) # Front Right
    #         self.__propellerList[1].setVelocity(front_left_motor_input) # Front Left
    #         self.__propellerList[2].setVelocity(rear_right_motor_input) # Rear Right
    #         self.__propellerList[3].setVelocity(-rear_left_motor_input) # Rear Left
        

    #     self.__propellerList[0].setVelocity(5) # Front Right
    #     self.__propellerList[1].setVelocity(5) # Front Left
    #     self.__propellerList[2].setVelocity(5) # Rear Right
    #     self.__propellerList[3].setVelocity(5) # Rear Left

    def step(self):
        rclpy.spin_once(self.subscription, timeout_sec=0)

        #Where is the drone?
        droneRoll, dronePitch, droneYaw = self.computer.getRollPitchYaw()
        positionX, positionY, vertical = self.gps.getValues()
        droneVelocity = self.gps.getSpeed()
        rollVelocity, pitchVelocity, yawVelocity = self.gyro.getValues()
        print("Roll, Pitch, Yaw: " + str(droneRoll) + str(dronePitch) + str(droneYaw))
        print("PosX, PosY, Vertical" + str(positionX) + str(positionY) + str(vertical))  

        if vertical < 25:
            roll_input = 50 * clamp(droneRoll, -1, 1) + rollVelocity
            pitch_input = 30.0 * clamp(dronePitch, -1, 1) + pitchVelocity
            yaw_input = 0
            clamped_difference_altitude = clamp(self.target_altitude - vertical + 0.6, -1, 1)
            vertical_input = 3.0 * pow(clamped_difference_altitude, 3.0)

            front_left_motor_input = 70 + vertical_input - yaw_input + pitch_input - roll_input
            front_right_motor_input = 70 + vertical_input + yaw_input + pitch_input + roll_input
            rear_left_motor_input = 70 + vertical_input + yaw_input - pitch_input - roll_input
            rear_right_motor_input = 70 + vertical_input - yaw_input - pitch_input + roll_input
            self.__propellerList[0].setVelocity(-front_right_motor_input) # Front Right
            self.__propellerList[1].setVelocity(front_left_motor_input) # Front Left
            self.__propellerList[2].setVelocity(rear_right_motor_input) # Rear Right
            self.__propellerList[3].setVelocity(-rear_left_motor_input) # Rear Left
        if vertical > 25:
            self.__propellerList[0].setVelocity(-68.5) # Front Right
            self.__propellerList[1].setVelocity(68.5) # Front Left
            self.__propellerList[2].setVelocity(68.5) # Rear Right
            self.__propellerList[3].setVelocity(-68.5) # Rear Left

def main():
    rclpy.init()
    robotControl = DroneDriver()
    print("Here!")
    rclpy.spin(robotControl)
    #robotControl.moveRobot()
    robotControl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()