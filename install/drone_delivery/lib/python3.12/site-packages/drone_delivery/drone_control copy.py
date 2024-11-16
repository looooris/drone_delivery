import rclpy
from geometry_msgs.msg import Twist
from controller import GPS, Gyro, InertialUnit

#HALF_DISTANCE_BETWEEN_WHEELS = 0.045
#WHEEL_RADIUS = 0.025
TAKEOFF_VELOCITY = 75

class DroneDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__camera = self.__robot.getDevice('camera')
        self.__gps = self.__robot.getDevice('gps')
        self.__internalcomputer = self.__robot.getDevice('inertial unit')
        self.__gyro = self.__robot.getDevice('gyro') # Sets up fundamental objects

        self.__gps.enable('1')
        self.__gyro.enable('1')
        self.__internalcomputer.enable('1')

        # Initalise Propellers
        self.__propellerList = [
            self.__robot.getDevice('front right propeller'),
            self.__robot.getDevice('front left propeller'),
            self.__robot.getDevice('rear right propeller'),
            self.__robot.getDevice('rear left propeller')
        ]
        for propeller in self.__propellerList:
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

        #Where is the drone?
        droneRoll, dronePitch, droneYaw = self.__internalcomputer.getRollPitchYaw()
        positionX, positionY, vertical = self.__gps.getValues()
        droneVelocity = self.__gps.getSpeed()
        rollVelocity, pitchVelocity, yawVelocity = self.__gyro.getValues()
        #print("Roll, Pitch, Yaw: " + str(droneRoll) + str(dronePitch) + str(droneYaw))
        #print("PosX, PosY, Vertical" + str(positionX) + str(positionY) + str(vertical))

        if vertical < 25:
            self.__propellerList[0].setVelocity(-75) # Front Right
            self.__propellerList[1].setVelocity(70) # Front Left
            self.__propellerList[2].setVelocity(75) # Rear Right
            self.__propellerList[3].setVelocity(-75) # Rear Left
        else:
            self.__propellerList[0].setVelocity(0) # Front Right
            self.__propellerList[1].setVelocity(0) # Front Left
            self.__propellerList[2].setVelocity(0) # Rear Right
            self.__propellerList[3].setVelocity(0) # Rear Left

rclpy.init()
robotControl = DroneDriver()
rclpy.spin(robotControl)
robotControl.destroy_node()
rclpy.shutdown()

