import rclpy
import math
import csv
import datetime
from rclpy.node import Node
import os
from sensor_msgs.msg import Imu

import rclpy.timer

class RecNode(Node):

    def __init__(self):
        super().__init__('rec_node')

        # Path to save csv file
        self.output_csv_path = '/home/ninad/Documents/accel_data/acceleration_data.csv'
        
        # Check if file exists
        self.file_exists = os.path.exists(self.output_csv_path)

        # Open CSV file for writing
        self.csv_file = open(self.output_csv_path, mode='a', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        if not self.file_exists:
            self.csv_writer.writerow(['Timestamp', 'Acceleration'])
        
        # Initialize machine state
        self.machine = True
        self.acc_magnitude = 9.81
        
        # Create subscription to IMU data
        self.subscription_Imu = self.create_subscription(
            Imu,
            '/olive/imu/id01/filtered_imu',
            self.listener_callback,
            10)
        
        # Create timers to perform the operation every "x" seconds
        self.timer1 = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('Hello from Olive EDGE!')

    def listener_callback(self, msg):
        # Check if acceleration values are outside the threshold range
        threshold_x = msg.linear_acceleration.x < 9.81 or msg.linear_acceleration.x > 9.81
        threshold_y = msg.linear_acceleration.y < 9.81 or msg.linear_acceleration.y > 9.81
        threshold_z = msg.linear_acceleration.z < 9.81 or msg.linear_acceleration.z > 9.81

        if (threshold_x or threshold_y or threshold_z):
            # Calculate the magnitude of acceleration
            self.acc_magnitude = math.sqrt(msg.linear_acceleration.x**2 + msg.linear_acceleration.y**2 + msg.linear_acceleration.z**2)
        else:
            self.acc_magnitude = 9.81

    def timer_callback(self):
        self.previous_machine_state = self.machine
        
        if self.acc_magnitude > 10.0 or self.acc_magnitude < 9.6:
            self.machine = True
        else:
            self.machine = False
        
        # Log machine state change
        if self.previous_machine_state != self.machine:
            if self.machine:
                self.get_logger().info('The machine is on!')
            else:
                self.get_logger().info('The machine is off!')

        # Set the actual time
        self.current_time = datetime.datetime.today().strftime("%Y-%m-%d %H:%M:%S")
       
        # Write data to CSV if the machine is on
        if self.machine:
            self.csv_writer.writerow([self.current_time, self.acc_magnitude])
            self.csv_file.flush()
            os.fsync(self.csv_file.fileno())
            self.get_logger().info('Writing to CSV: Timestamp: %s, Acceleration: %.2f m/s²' % (self.current_time, self.acc_magnitude))
            self.acc_magnitude = 9.81

    def destroy_node(self):
        # Close CSV file when node is shutting down
        self.csv_file.close()
        super().destroy_node()
        
        
def main(args=None):
    rclpy.init(args=args)
    try:
        rec_node = RecNode()    
        rclpy.spin(rec_node)
    except KeyboardInterrupt:
        pass
    finally:
        rec_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
