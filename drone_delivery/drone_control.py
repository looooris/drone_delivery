import rclpy
<<<<<<< Updated upstream
from geometry_msgs.msg import Twist
=======
from geometry_msgs.msg import Twist, Point
from webots_ros2_msgs.srv import GetBool
from controller import Robot
import tkinter
import threading
import queue
data_queue = queue.Queue()

def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)
>>>>>>> Stashed changes

#HALF_DISTANCE_BETWEEN_WHEELS = 0.045
#WHEEL_RADIUS = 0.025

class droneControl:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__camera = self.__robot.getDevice('camera')

        #rclpy.init(args=None)
        self.__node = rclpy.create_node('drone_control')
        #self.__node.create_subscription(None, 'cmd_vel', self.__cmd_vel_callback, 1)

    #def __cmd_vel_callback(self, twist):
        #self.__target_twist = twist

    #def step(self):
        #rclpy.spin_once(self.__node, timeout_sec=0)

        #forward_speed = self.__target_twist.linear.x
        #angular_speed = self.__target_twist.angular.z

        #command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        #command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

<<<<<<< Updated upstream
        #self.__left_motor.setVelocity(command_motor_left)
        #self.__right_motor.setVelocity(command_motor_right)
=======
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
        data_queue.put(gps_values)  # Put the GPS values in the queue to send to the GUI
        droneVelocity = self.gps.getSpeed()
        gyroValues = self.gyro.getValues()
        distSense = self.distsensor.getValue()
        print(distSense)
        return internalComputerValues, gpsValues, gyroValues, droneVelocity, distSense

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
        #print(msg.x, msg.y, msg.z)

    def calcPitchRoll(self, gps):
        diff_x = gps[0] - self.robot_target.linear.x
        # 0 - 2
        diff_y = gps[1] - self.robot_target.linear.y

        rm = 0
        pm = 0

        if diff_x > 0:
            pm = max(-1.5, -diff_x)
        elif diff_x < 0:
            pm = min(1.5, -diff_x)

        if diff_y > 0:
            rm = min(1.5, diff_y)
        elif diff_y < 0:
            rm = max(-1.5, diff_y)

        return rm, pm

    def step(self): 
        rclpy.spin_once(self.subscription, timeout_sec=0)
        #self.__target_twist = Twist()
        intComVal, gpsVal, gyroVal, droneVelocity, distanceSensor = self.locateDrone()
        update_event.set()  # Notify GUI thread to update
        
        if self.launchable: # Robot is in a launchable state
            roll_move = 0
            pitch_move = 0
            vertical_input = 0
            fineControl = True
            if abs(gpsVal[1] - self.robot_target.linear.y) < 0.25 and abs(gpsVal[0] - self.robot_target.linear.x) < 0.25: # Robot is at position
                if abs(gpsVal[2] - self.robot_target.linear.z) < 0.1:
                    self.at_target = True
                    for propeller in self.propList:
                        propeller.setVelocity(0)
                    fineControl = False
                else:   
                    if gpsVal[2] > self.robot_target.linear.z:
                        #print('Robot is at position but needs to decend')
                        self.target_altitude = min(self.robot_target.linear.z, 0)
                        roll_move, pitch_move = self.calcPitchRoll(gpsVal)
                    else:
                        pass
                        #print('Robot is at position but needs to ascend')
                    
                    
                    vertical_input = clamp(self.target_altitude-gpsVal[2], -5, -2   )
                
            elif gpsVal[2] < self.target_altitude: # Robot is not yet at height
                #print('Robot is not yet at height')
                vertical_input = 3.0 * clamp(self.target_altitude - gpsVal[2] + 0.6, -1, 1)**3.0
            else: 
                roll_move, pitch_move = self.calcPitchRoll(gpsVal)
                vertical_input = 3.0 * clamp(self.target_altitude - gpsVal[2] + 0.6, -1, 1)**3.0                
                      
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

#trying
def update_gui(gps_values):
    position_label.config(text=f"Position: x={gps_values[0]:.2f}, y={gps_values[1]:.2f}")

# Tkinter GUI function to display the GPS values
def tkinter_thread():
    "iran"
    m = tkinter.Tk()
    m.title("Drone Position")

    # Label to display the position of the robot
    global position_label
    position_label = tkinter.Label(m, text="Position: x=0, y=0")
    position_label.pack()

    def update_position():
        while True:
            gps_values = data_queue.get()  # Block until data is available from the queue
            update_gui(gps_values)  # Update the GUI with the latest GPS data
    
    # Run the update_position function in the background thread
    threading.Thread(target=update_position, daemon=True).start()

    m.mainloop()

def main():
    #rclpy.init(args=None)
    t = threading.Thread(target=tkinter_thread)
    t.start()
    robotControl = DroneDriver()
    rclpy.spin(robotControl)
    robotControl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
>>>>>>> Stashed changes
