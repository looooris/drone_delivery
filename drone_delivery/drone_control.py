import rclpy
from geometry_msgs.msg import Twist, Point
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

        self.targetZ = 0
        self.targetX = 0
        self.targetY = 0

        self.__target_twist = Twist()

        # Configure ROS interface
        rclpy.init(args=None)
        self.subscription = rclpy.create_node('driver_'+self.name) 
        self.subscription.create_subscription(Twist, 'cmd_vel_' + self.name, self.__cmd_vel_callback, 1)
        self.subscription.create_subscription(Point, 'goto_robot', self.listener_position, 1)

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

    def listener_position(self, msg):
        self.targetX = msg.x
        self.targetY =  msg.y
        self.targetZ = msg.z + 3 # this needs to calculate +3 from either higher start or end, but not constantly update
        #print(msg.x, msg.y, msg.z)

    def goToLocation(self, targetLocation):
        # Pick up item
        for waypoint in targetLocation:
            gpsValues = self.gps.getValues() # Initial Drone Location
            self.targetZ = (max(gpsValues[2], waypoint[2])) + 3
            #print(max(gpsValues[2], waypoint[2]) +3)
            print("bello!")
            self.targetX = gpsValues[0]
            self.targetY = gpsValues[1]
            while abs(self.gps.getValues()[2] - self.targetZ) > 0.25: # Get to height
                rclpy.spin_once(robotControl)

            self.targetX = waypoint[0]
            self.targetY = waypoint[1]
            while abs(self.gps.getValues[0] - self.targetX) > 0.05 and abs(self.gps.getValues[1] - self.targetY) > 0.05: # Go to destination
                rclpy.spin_once(robotControl)


            self.targetZ = waypoint[2] # lower to target
            while abs(self.gps.getValues()[2] - self.targetZ) > 0.25: # Lower height
                rclpy.spin_once(robotControl)
        
        #drop item


    #def step(self):
        #print("bello!")

    def step(self): #having difficulties with the name 'step'...io
        rclpy.spin_once(self.subscription, timeout_sec=0)
        self.__target_twist = Twist()
        #self.targetX = 2
        #self.targetY = 2
        #self.targetZ = 2
        #print(self.targetX, self.targetY, self.targetZ)
        #print(self.gps.getValues()[2] - self.targetZ)

        #Where is the drone?
        intComVal, gpsVal, gyroVal, droneVelocity = self.locateDrone()

        #Read target from topic?

        forward = None
        right = None

        # X Positioning
        if abs(gpsVal[0] - self.targetX) > 0.25:
            xMovement = gpsVal[0] - self.targetX
            if xMovement < 0:
                right = True
            else:
                right = False
        # Y Positioning
        if abs(gpsVal[1] - self.targetY) > 0.25:
            yMovement = gpsVal[1] - self.targetY
            if yMovement < 0:
                forward = True
            else:
                forward = False
        
        
        #print("Roll, Pitch, Yaw: " + str(intComVal[0]) + " " + str(intComVal[1]) + " " + str(intComVal[2]))
        #print("PosX, PosY, PosZ: " + str(gpsVal[0]) + " " + str(gpsVal[1]) + " " + str(gpsVal[2]))
         

        roll_input = 50 * clamp(intComVal[0], -1, 1) + gyroVal[0] 
        pitch_input = 30.0 * clamp(intComVal[1], -1, 1) + gyroVal[1]
        yaw_input = 2 * (self.__target_twist.angular.z - gyroVal[2])
        clamped_difference_altitude = clamp(self.targetZ - gpsVal[2] + 0.6, -1, 1)
        vertical_input = 3.0 * clamped_difference_altitude**3.0


        if gpsVal[2] > self.targetZ - 0.25 and abs(self.targetX - gpsVal[0]) > 0.5:
            #print("Forward, Right: "+ str(forward) + " " + str(right)) 
            if right:
                pitch_input = pitch_input - 1
            else:
                pitch_input = pitch_input + 1
            if forward:
                roll_input = roll_input - 1
            else:
                roll_input = roll_input + 1

        front_left_motor_input = 70 + vertical_input - yaw_input + pitch_input - roll_input
        front_right_motor_input = 70 + vertical_input + yaw_input + pitch_input + roll_input
        rear_left_motor_input = 70 + vertical_input + yaw_input - pitch_input - roll_input
        rear_right_motor_input = 70 + vertical_input - yaw_input - pitch_input + roll_input
        self.frp.setVelocity(-front_right_motor_input) # Front Right
        self.flp.setVelocity(front_left_motor_input) # Front Left
        self.rrp.setVelocity(rear_right_motor_input) # Rear Right
        self.rlp.setVelocity(-rear_left_motor_input) # Rear Left

        #self.__propellerList[0].setVelocity(-100) # Front Right
        #self.__propellerList[1].setVelocity(100) # Front Left
        #self.__propellerList[2].setVelocity(100) # Rear Right
        #self.__propellerList[3].setVelocity(-100) # Rear Left

def main():
    #rclpy.init(args=None)
    robotControl = DroneDriver()
    target = [[2, 2, 0]]
    robotControl.goToLocation(target)
    #rclpy.spin(robotControl)
    #robotControl.moveRobot()
    robotControl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()