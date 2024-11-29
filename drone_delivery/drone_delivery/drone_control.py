import rclpy
from rclpy.action import ActionServer
from geometry_msgs.msg import Point
from controller import Robot
import math
import time

from drone_delivery_services.srv import Destination, Gripper
from drone_delivery_services.msg import Droneloc, Emergency
    
class DroneDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        self.time_step = 1
        self.start_time = time.time() # initalize a timer

        # Initalise sensors
        self.distanceSensor = self.robot.getDevice('distance sensor')
        self.gps = self.robot.getDevice('gps')
        self.computer = self.robot.getDevice('inertial unit')
        self.gyro = self.robot.getDevice('gyro') 

        # Enable sensors
        self.distanceSensor.enable(self.time_step)
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

        # Robot metadata
        self.is_gripper_open = True 
        self.launchable = False
        self.at_target = False
        self.killRobot = False
        self.safe = True
        

        # Destination metadata
        self.robot_target = Point()
        self.target_altitude = 0
        self.to_pharmacy = False

        # Configure ROS interface
        rclpy.init(args=None)
        self.subscription = rclpy.create_node('driver_'+self.robot.name) 
        self.destination_client = self.subscription.create_client(Destination, 'drone_destination_service')
        self.gripper_client = self.subscription.create_client(Gripper, self.robot.name + '_gripper')
        self.destrequest = Destination.Request()
        self.griprequest = Gripper.Request()
        while not self.destination_client.wait_for_service(timeout_sec=1.0) or not self.gripper_client.wait_for_service(timeout_sec=1.0):
            self.subscription.get_logger().info('service(s) not available, waiting again...')

        self.location_publisher = self.subscription.create_publisher(Droneloc, 'drone_location', 1)
        self.emergency_stop = self.subscription.create_subscription(Emergency, 'drone_emergency', self.emergency_callback, 1)

        self.location_publisher_timer = self.subscription.create_timer(3, self.location_callback)

    def initiate_emergency(self, msg):
        if not msg.safe:
            self.safe = False
            self.subscription.get_logger().info("collission imminent. taking evasive action.")
        else:   
            self.safe = True

    def emergency_callback(self, msg):
        if self.robot.name == "drone_one" and msg.id == 1: self.initiate_emergency(msg)
        elif self.robot.name == "drone_two" and msg.id == 2: self.initiate_emergency(msg)
    
    async def location_callback(self):
        gpsValues = self.gps.getValues()
        message = Droneloc()

        message.currentposition = Point()
        message.currentposition.x = gpsValues[0]
        message.currentposition.y = gpsValues[1]
        message.currentposition.z = gpsValues[2]

        if self.robot.name == "drone_one":
            message.id = 1
        else:
            message.id = 2

        self.location_publisher.publish(message)

    # For sending a request to the location publisher. Sends current position and receives goal
    def sendRequest(self):
        gpsValues = self.gps.getValues()
        self.destrequest.droneid = self.robot.name
        self.destrequest.currentposition.x = gpsValues[0]
        self.destrequest.currentposition.y = gpsValues[1]
        self.destrequest.currentposition.z = gpsValues[2]
        self.destrequest.starttime = self.start_time # time at beginning of trip
        self.destfuture = self.destination_client.call_async(self.destrequest)
        self.reset_time()
        rclpy.spin_until_future_complete(self.subscription, self.destfuture)
        return self.destfuture.result()

    # Sends a request to open the gripper. Sends goal
    def sendGripRequest(self, data):
        self.griprequest.open = data
        self.is_gripper_open = data
        self.gripfuture = self.gripper_client.call_async(self.griprequest)
        rclpy.spin_until_future_complete(self.subscription, self.gripfuture)
        return self.gripfuture.result()

    # Configures destination values
    def updateTarget(self, data):
        if [data.deliverylocation.x, data.deliverylocation.y, data.deliverylocation.z] == [-1, -1, -1]:
            self.killRobot = True
            self.subscription.get_logger().info(self.robot.name + " finished!")
        else:
            self.robot_target.x = data.deliverylocation.x
            self.robot_target.y =  data.deliverylocation.y
            if data.deliverylocation.z < 0:
                raise Exception("Z cannot be less than 0")
            self.robot_target.z = data.deliverylocation.z
            self.target_altitude = max(3, data.deliverylocation.z + 3)
            self.to_pharmacy = data.pharmacy
            self.subscription.get_logger().info('New ' + str(self.robot.name) + ' Target: '+ str(self.robot_target.x) + ' ' + ' ' + str(self.robot_target.y) + ' ' + ' ' + str(self.robot_target.z) + ' ' + ' ' + str(self.to_pharmacy))
        
    # Returns the location, angle, speed and distance sensor detections of the drone
    def locateDrone(self):
        internalComputerValues = self.computer.getRollPitchYaw()
        gpsValues = self.gps.getValues()
        droneVelocity = self.gps.getSpeed()
        gyroValues = self.gyro.getValues()
        distSense = self.distanceSensor.getValue()
        return internalComputerValues, gpsValues, gyroValues, droneVelocity, distSense

    # Calulates if the drone needs to increase or decrease pitch and roll
    def calcPitchRoll(self, gps):
        diff_x = gps[0] - self.robot_target.x
        diff_y = gps[1] - self.robot_target.y

        rm = 0
        pm = 0

        if diff_x > 0:
            pm = max(-1, -diff_x) # increases pitch
        elif diff_x < 0:
            pm = min(1, -diff_x)  # decreases pitch

        if diff_y > 0:
            rm = min(1, diff_y)   # increases roll
        elif diff_y < 0:
            rm = max(-1, diff_y)  # decreases roll

        return rm, pm

    # Calculates the drone's distance and angle to final destination
    def calcAngleDist(self, gps, imu):
        diff_x = self.robot_target.x - gps[0] 
        diff_y = self.robot_target.y - gps[1]    
        distance = math.sqrt((diff_x)**2 + (diff_y)**2)

        angleToMove = math.atan2((diff_y),(diff_x)) - imu[2] 

        if (angleToMove >  math.pi): # return negative if less than pi, so robot turns other way
            angleToMove -= 2 * math.pi

        return distance, angleToMove

    # returns a value bound by two other values
    def bind(self, value, value_min, value_max):
        if value < value_min:
            return value_min
        elif value > value_max:
            return value_max
        else:
            return value
        
    def reset_time(self):
        self.start_time = time.time()

    def step(self): 
        if not self.killRobot:
            rclpy.spin_once(self.subscription, timeout_sec=0)
            intComVal, gpsVal, gyroVal, droneVelocity, distSense = self.locateDrone()
            if self.launchable: # Robot is in a launchable state
                roll_move = 0
                pitch_move = 0
                vertical_input = 0
                yaw_input = 0
                fineControl = False

                distance, angle = self.calcAngleDist(gpsVal, intComVal)
                roll_move, pitch_move = self.calcPitchRoll(gpsVal)
                if not self.safe: #if near collision, stop robot
                    yaw_input = 0     
                    roll_input = 50 * self.bind(intComVal[0], -1, 1) 
                    pitch_input = 30 * self.bind(intComVal[1], -1, 1)
                    vertical_input = 5* self.bind(self.target_altitude, -1, 1)**3.0
                elif distance < 1:   
                    #landing sequence
                    if abs(gpsVal[1] - self.robot_target.y) < 0.5 and abs(gpsVal[0] - self.robot_target.x) < 0.5: 
                        if abs(gpsVal[2] - self.robot_target.z) < 0.1:  # turn off propellers
                            self.at_target = True
                            self.launchable = False
                            for propeller in self.propList:
                                propeller.setVelocity(0)    # stops propellers
                            fineControl = True  # disables movement
                            if self.to_pharmacy:
                                if self.is_gripper_open:
                                    self.sendGripRequest(False) # closes gripper if destination is pharmacy (pick up)
                            else:
                                if not self.is_gripper_open:
                                    self.sendGripRequest(True)  # opens gripper if destination is not pharmacy (drop off)
                        else:   
                            if gpsVal[2] > self.robot_target.z:
                                self.target_altitude = min(self.robot_target.z, 0)  # begins descent
                                vertical_input = self.bind(self.target_altitude-gpsVal[2], -4, -1)
                            else:
                                vertical_input = 3.0 * self.bind(self.target_altitude - gpsVal[2] + 0.6, -1, 1)**3.0    # begins ascent    
                    if not fineControl:
                        yaw_input = 0.8 * angle / (2 * math.pi)     
                        roll_input = 50 * self.bind(intComVal[0], -1, 1) + gyroVal[0]
                        pitch_input = 30 * self.bind(intComVal[1], -1, 1) + gyroVal[1] + self.bind(math.log10(abs(angle)), -0.2, 0.1)
                                                    
                        
                else:
                    #self.subscription.get_logger().info('Distance Sensor Reads '+ str(distSense) + '. Target Altitude ' + str(self.target_altitude))
                    if distSense > 40: 
                        # ascends if foreign object detected within 30 units (2m)
                        self.target_altitude += 0.5
                        pitch_input = 30 * self.bind(intComVal[1], -1, 1) + gyroVal[1]
                    else:
                        # if robot is clear but still has a lot to travel upwards, cancel the upwards movement
                        if distSense == 0 and abs(self.target_altitude - gpsVal[2]) > 2 and self.target_altitude != 3:
                            if self.target_altitude != self.robot_target.z + 3:
                                self.subscription.get_logger().info('New target altitude at ' + str(gpsVal[2]) + ' Distance Sensor Reads '+ str(distSense) + '. Target Altitude ' + str(self.target_altitude))
                                self.target_altitude = gpsVal[2]

                        # Mathematical calculations - based upon https://github.com/patrickpbarroso/drone-simulation
                        if abs(angle) > 0.1:
                            yaw_input = 2 * angle / (2 * math.pi)
                            pitch_input = 30 * self.bind(intComVal[1], -1, 1) + gyroVal[1]
                        else:
                            #slows rotation
                            yaw_input = angle / (2 * math.pi)
                            pitch_input = 30 * self.bind(intComVal[1], -1, 1) + gyroVal[1] + self.bind(math.log10(abs(angle)), -1, 0.1)      

                    vertical_input = 3.0 * self.bind(self.target_altitude - gpsVal[2] + 0.6, -1, 1)**3.0
                    roll_input = 50 * self.bind(intComVal[0], -1, 1) + gyroVal[0]
    

                if not fineControl:
                    # Robot propeller settings
                    self.frp.setVelocity(-(70 + vertical_input + yaw_input + pitch_input + roll_input)) # Front Right
                    self.flp.setVelocity(70 + vertical_input - yaw_input + pitch_input - roll_input) # Front Left
                    self.rrp.setVelocity(70 + vertical_input - yaw_input - pitch_input + roll_input) # Rear Right
                    self.rlp.setVelocity(-(70 + vertical_input + yaw_input - pitch_input - roll_input)) # Rear Left

            else:
                if gpsVal[2] < 0.25: # drone on the ground
                    response = None
                    while response is None:
                        response = self.sendRequest()

                    

                    # update target and relaunch robot
                    self.updateTarget(response)              
                    self.launchable = True 

                else:
                    raise Exception("The drone isn't launchable, it's not on the ground")

def main():
    robotControl = DroneDriver()
    rclpy.spin(robotControl)
    robotControl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()