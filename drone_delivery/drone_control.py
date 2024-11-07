import rclpy
from geometry_msgs.msg import Twist, Point
from webots_ros2_msgs.srv import GetBool
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

        # Initalise sensors
        self.camera = self.robot.getDevice('camera')
        self.gps = self.robot.getDevice('gps')
        self.computer = self.robot.getDevice('inertial unit')
        self.gyro = self.robot.getDevice('gyro') 

        # Enable sensors
        self.camera.enable(self.time_step)
        self.gps.enable(self.time_step)
        self.gyro.enable(self.time_step)
        self.computer.enable(self.time_step)

        # Initalise propellers
        self.frp = self.robot.getDevice('front right propeller')
        self.flp = self.robot.getDevice('front left propeller')
        self.rrp = self.robot.getDevice('rear right propeller')
        self.rlp = self.robot.getDevice('rear left propeller')
        self.propList = [self.frp, self.flp, self.rrp, self.rlp]

        # Set initial velocity to zero
        for propeller in self.propList:
            propeller.setPosition(float('inf'))
            propeller.setVelocity(0)

        self.target_altitude = 0
        self.lin_x_int = 0
        self.lin_y_int = 0

        self.item_grabbed = False # Robot metadata
        self.launchable = False
        self.at_target = False

        self.robot_target = Twist()
        self.vertical_ref = 1

        #self.__target_twist = Twist()

        # Configure ROS interface
        rclpy.init(args=None)
        self.subscription = rclpy.create_node('driver_'+self.name) 
        self.subscription.create_subscription(Twist, self.name + '/cmd_vel', self.__cmd_vel_callback, 1)
        self.subscription.create_subscription(Point, 'goto_robot', self.listener_position, 1)

        #self.voicing = rclpy.create_node('publisher_'+self.name) 
        self.metadata_publisher = self.subscription.create_publisher(Point, 'robot_metadata', 1)
        self.subscription.create_timer(1, self.metadata_pub)

    def __cmd_vel_callback(self, twist):
        self.robot_target = twist

    def locateDrone(self):
        internalComputerValues = self.computer.getRollPitchYaw()
        gpsValues = self.gps.getValues()
        droneVelocity = self.gps.getSpeed()
        gyroValues = self.gyro.getValues()
        return internalComputerValues, gpsValues, gyroValues, droneVelocity

    def metadata_pub(self):
        msg = Point()
        if self.launchable:
            msg.x = float(1) # Launchable
        else:
            msg.x = float(0)
        if self.item_grabbed:
            msg.y = float(1) # Item Grabbed
        else:
            msg.y = float(0)
        if self.at_target: 
            msg.z = float(1) # At Target
        else:
            msg.z = float(0)
        self.metadata_publisher.publish(msg)

    def bindValue(self, value):
        return min(max(value, -1), 1)

    def listener_position(self, msg):
        self.robot_target.linear.x = msg.x
        self.robot_target.linear.y =  msg.y
        #if msg.z < 0:
            #raise Exception("Z cannot be less than 0")
        self.robot_target.linear.z = msg.z # this needs to calculate +3 from either higher start or end, but not constantly update
        self.target_altitude = max(3, msg.z)
        print(msg.x, msg.y, msg.z)

    def calcPitchRoll(self, gps):
        diff_x = gps[0] - self.robot_target.linear.x
        # 0 - 2
        diff_y = gps[1] - self.robot_target.linear.y

        rm = 0
        pm = 0

        if diff_x > 0:
            pm = max(-1, -diff_x)
        elif diff_x < 0:
            pm = min(1, -diff_x)

        if diff_y > 0:
            rm = min(1, diff_y)
        elif diff_y < 0:
            rm = max(-1, diff_y)

        return rm, pm

    def step(self): 
        rclpy.spin_once(self.subscription, timeout_sec=0)
        #self.__target_twist = Twist()
        intComVal, gpsVal, gyroVal, droneVelocity = self.locateDrone()
        if self.launchable: # Robot is in a launchable state
            roll_move = 0
            pitch_move = 0
            vertical_input = 0
            fineControl = True
            if abs(gpsVal[1] - self.robot_target.linear.y) < 0.25 and abs(gpsVal[0] - self.robot_target.linear.x) < 0.25: # Robot is at position
                if abs(gpsVal[2] - self.robot_target.linear.z) < 0.1:
                    # i = 50
                    # while abs(gpsVal[2] - self.robot_target.linear.z) < 0.1:
                    #     self.killProp(i)
                    #     i -= 1
                    #self.killProp(0)
                    self.at_target = True
                    #self.launchable = False
                    print("Robot is at its target :3")
                    for propeller in self.propList:
                        propeller.setVelocity(0)
                    fineControl = False
                   
                else:   
                    if gpsVal[2] > self.robot_target.linear.z:
                        print('Robot is at position but needs to decend')
                    else:
                        print('Robot is at position but needs to ascend')
                    self.target_altitude = min(self.robot_target.linear.z, 0)
                    roll_move, pitch_move = self.calcPitchRoll(gpsVal)
                    
                    vertical_input = clamp(self.target_altitude-gpsVal[2], -5, -2   )
                
            elif gpsVal[2] < self.target_altitude: # Robot is not yet at height
                print('Robot is not yet at height')
                vertical_input = 3.0 * clamp(self.target_altitude - gpsVal[2] + 0.6, -1, 1)**3.0
            else: 
                roll_move, pitch_move = self.calcPitchRoll(gpsVal)
                vertical_input = 3.0 * clamp(self.target_altitude - gpsVal[2] + 0.6, -1, 1)**3.0                
                #print(diff_x, diff_y)
                #print(pitch_move)
                      
            # Mathematical calculations - based upon https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_mavic/webots_ros2_mavic/mavic_driver.py and used with Apache 2.0 Licence
            if fineControl:
                    roll_input = 50 * clamp(intComVal[0], -1, 1) + gyroVal[0] + roll_move
                    pitch_input = 30 * clamp(intComVal[1], -1, 1) + gyroVal[1] + pitch_move
                    yaw_input = 2 * (self.robot_target.angular.z - gyroVal[2])

                    # Robot propeller settings
                    self.frp.setVelocity(-(70 + vertical_input + yaw_input + pitch_input + roll_input)) # Front Right
                    self.flp.setVelocity(70 + vertical_input - yaw_input + pitch_input - roll_input) # Front Left
                    self.rrp.setVelocity(70 + vertical_input - yaw_input - pitch_input + roll_input) # Rear Right
                    self.rlp.setVelocity(-(70 + vertical_input + yaw_input - pitch_input - roll_input)) # Rear Left
                    #Read target from topic?

        else:
            if gpsVal[2] < 0.25: # drone on da ground !
                print("picking up item...")
                self.launchable = True
                self.item_grabbed = True # Need to check if item is available at location
            else:
                print("the drone isn't launchable. it's not on the ground")

def main():
    #rclpy.init(args=None)
    robotControl = DroneDriver()
    rclpy.spin(robotControl)
    robotControl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()