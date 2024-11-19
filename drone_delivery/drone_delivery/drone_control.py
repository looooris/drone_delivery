import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from geometry_msgs.msg import Point
from webots_ros2_msgs.srv import GetBool
from controller import Robot
import math
import time

from drone_delivery_services.srv import Destination, Gripper
    
def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)

class DroneDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        self.time_step = 1
        self.name = "drone_one"

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

        # Destination metadata
        self.robot_target = Point()
        self.target_altitude = 0
        self.to_pharmacy = False

        # Configure ROS interface
        rclpy.init(args=None)
        self.subscription = rclpy.create_node('driver_'+self.name) 
        self.destination_client = self.subscription.create_client(Destination, 'drone_destination')
        self.gripper_client = self.subscription.create_client(Gripper, 'drone_gripper')
        self.destrequest = Destination.Request()
        self.griprequest = Gripper.Request()
        while not self.destination_client.wait_for_service(timeout_sec=1.0) or not self.gripper_client.wait_for_service(timeout_sec=1.0):
            self.subscription.get_logger().info('service(s) not available, waiting again...')

    def sendRequest(self):
        gpsValues = self.gps.getValues()
        self.destrequest.currentposition.x = gpsValues[0]
        self.destrequest.currentposition.y = gpsValues[1]
        self.destrequest.currentposition.z = gpsValues[2]
        self.destfuture = self.destination_client.call_async(self.destrequest)
        rclpy.spin_until_future_complete(self.subscription, self.destfuture)
        return self.destfuture.result()

    def sendGripRequest(self, data):
        self.griprequest.open = data
        self.is_gripper_open = data
        self.gripfuture = self.gripper_client.call_async(self.griprequest)
        rclpy.spin_until_future_complete(self.subscription, self.gripfuture)
        return self.gripfuture.result()

    def updateTarget(self, data):
        self.robot_target.x = data.deliverylocation.x
        self.robot_target.y =  data.deliverylocation.y
        if data.deliverylocation.z < 0:
            raise Exception("Z cannot be less than 0")
        self.robot_target.z = data.deliverylocation.z
        self.target_altitude = max(3, data.deliverylocation.z + 3)
        self.to_pharmacy = data.pharmacy
        self.subscription.get_logger().info('New Robot Target: '+ str(self.robot_target.x) + ' ' + ' ' + str(self.robot_target.y) + ' ' + ' ' + str(self.robot_target.z) + ' ' + ' ' + str(self.to_pharmacy))
        
    def locateDrone(self):
        internalComputerValues = self.computer.getRollPitchYaw()
        gpsValues = self.gps.getValues()
        droneVelocity = self.gps.getSpeed()
        gyroValues = self.gyro.getValues()
        distSense = self.distanceSensor.getValue()
        return internalComputerValues, gpsValues, gyroValues, droneVelocity, distSense

    def calcPitchRoll(self, gps):
        diff_x = gps[0] - self.robot_target.x
        # 0 - 2
        diff_y = gps[1] - self.robot_target.y

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

    def calcAngleDist(self, gps, imu):
        diff_x = self.robot_target.x - gps[0]
        diff_y = self.robot_target.y - gps[1]    
        distance = math.sqrt((diff_x)**2 + (diff_y)**2)

        angleToMove = math.atan2((diff_y),(diff_x)) - imu[2] 

        if (angleToMove >  math.pi):
            angleToMove -= 2 * math.pi

        # print("to Target")
        # print((math.atan2((diff_y),(diff_x)) * 180) / math.pi)
        # print("Current")
        # print((imu[2] * 180) / math.pi)
        return distance, angleToMove

    def step(self): 
        rclpy.spin_once(self.subscription, timeout_sec=0)
        #self.__target_twist = Twist()
        intComVal, gpsVal, gyroVal, droneVelocity, distSense = self.locateDrone()
        if self.launchable: # Robot is in a launchable state
            roll_move = 0
            pitch_move = 0
            vertical_input = 0
            yaw_input = 0
            fineControl = False

            distance, angle = self.calcAngleDist(gpsVal, intComVal)
            roll_move, pitch_move = self.calcPitchRoll(gpsVal)
            #if abs(gpsVal[1] - self.robot_target.linear.y) < 0.5 and abs(gpsVal[0] - self.robot_target.linear.x) < 0.5: # Robot is at position
            if distance < 1:   
                # need to adjust for the direction the robot faces
                if abs(gpsVal[1] - self.robot_target.y) < 0.5 and abs(gpsVal[0] - self.robot_target.x) < 0.5:
                    if abs(gpsVal[2] - self.robot_target.z) < 0.1:
                        self.at_target = True
                        self.launchable = False
                        for propeller in self.propList:
                            propeller.setVelocity(0)
                        fineControl = True
                        if self.to_pharmacy:
                            if self.is_gripper_open:
                                self.sendGripRequest(False)
                        else:
                            if not self.is_gripper_open:
                                self.sendGripRequest(True)
                        

                    else:   
                        if gpsVal[2] > self.robot_target.z:
                            #print('Robot is at position but needs to decend')
                            self.target_altitude = min(self.robot_target.z, 0)
                            vertical_input = clamp(self.target_altitude-gpsVal[2], -4, -1)
                        else:
                            #print('Robot is at position but needs to ascend') 
                            vertical_input = 3.0 * clamp(self.target_altitude - gpsVal[2] + 0.6, -1, 1)**3.0    
                if not fineControl:      
                    yaw_input = 0.8 * angle / (2 * math.pi)     
                    roll_input = 50 * clamp(intComVal[0], -1, 1) + gyroVal[0]
                    pitch_input = 30 * clamp(intComVal[1], -1, 1) + gyroVal[1] + clamp(math.log10(abs(angle)), -0.2, 0.1)
                                                 
                      
            # Mathematical calculations - based upon https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_mavic/webots_ros2_mavic/mavic_driver.py and used with Apache 2.0 Licence
            else:
                #self.subscription.get_logger().info('Distance Sensor Reads '+ str(distSense) + '. Target Altitude ' + str(self.target_altitude))
                if distSense > 30:
                    self.target_altitude += 0.5
                    roll_input = 50 * clamp(intComVal[0], -1, 1) + gyroVal[0]
                    pitch_input = 30 * clamp(intComVal[1], -1, 1) + gyroVal[1]
                else:
                    # if robot is clear but still has a lot to travel upwards, cancel the upwards movement
                    if distSense == 0 and abs(self.target_altitude - gpsVal[2]) > 2 and self.target_altitude != 3:
                        if self.target_altitude != self.robot_target.z + 3:
                            self.subscription.get_logger().info('New target altitude at ' + str(gpsVal[2]) + ' Distance Sensor Reads '+ str(distSense) + '. Target Altitude ' + str(self.target_altitude))
                            self.target_altitude = gpsVal[2]
                    if abs(angle) > 0.1:
                        yaw_input = 2 * angle / (2 * math.pi)     
                        pitch_input = 30 * clamp(intComVal[1], -1, 1) + gyroVal[1]
                    else:
                        yaw_input = 1 * angle / (2 * math.pi)
                        pitch_input = 30 * clamp(intComVal[1], -1, 1) + gyroVal[1] + clamp(math.log10(abs(angle)), -1, 0.1)      

                vertical_input = 3.0 * clamp(self.target_altitude - gpsVal[2] + 0.6, -1, 1)**3.0
                roll_input = 50 * clamp(intComVal[0], -1, 1) + gyroVal[0]
  

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

                self.updateTarget(response)              
                self.launchable = True 

            else:
                raise Exception("The drone isn't launchable, it's not on the ground")

def main():
    #rclpy.init(args=None)
    robotControl = DroneDriver()
    rclpy.spin(robotControl)
    #robotControl.destroy_node()
    #rclpy.shutdown()


if __name__ == '__main__':
    main()