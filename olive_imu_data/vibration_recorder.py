#!/usr/bin/env python3

import rclpy
import math
import csv
import datetime
from rclpy.node import Node
import os
from sensor_msgs.msg import Imu
from geometry_msgs.msg import AccelStamped
from dirsync import sync

class RecNode(Node):

    def __init__(self):
        super().__init__('rec_node')

        # Path to save CSV file
        self.output_csv_dir = '/home/guts/Documents/accel_data/'
        
        self.current_date = datetime.datetime.today().strftime("%Y-%m-%d")
        self.output_csv_path = os.path.join(self.output_csv_dir, f'{self.current_date}.csv')

        # Open CSV file for writing
        self.csv_file = open(self.output_csv_path, mode='a', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write the header if the file is new
        if os.stat(self.output_csv_path).st_size == 0:
            self.csv_writer.writerow(['Timestamp', 'Linear x', 'Linear y', 'Linear z', 'Filtered x', 'Filtered y', 'Filtered z', 'Angular x', 'Angular y', 'Angular z'])
        
        # Initialize machine state and other variables
        self.machine = True
        self.acc_magnitude = 9.81
        self.linear_x = self.linear_y = self.linear_z = 0.0
        self.filtered_x = self.filtered_y = self.filtered_z = 0.0
        
        # Create subscriptions
        self.subscription_Imu = self.create_subscription(
            Imu,
            '/olive/imu/id01/filtered_imu',
            self.imu_callback,
            10)
        
        self.subscription = self.create_subscription(
            AccelStamped,
            '/olive/imu/id01/linear_accel',
            self.accel_callback,
            10)
        
        # Create timers to perform the operation every "x" seconds
        self.timer1 = self.create_timer(1.0, self.timer_callback)
        self.timer2 = self.create_timer(60.0, self.sync_data)

        self.get_logger().info('Hello from Olive EDGE!')

    def imu_callback(self, msg):
        # Extract filtered linear acceleration from the IMU message
        self.filtered_x = msg.linear_acceleration.x
        self.filtered_y = msg.linear_acceleration.y
        self.filtered_z = msg.linear_acceleration.z
        self.angular_x = msg.angular_velocity.x
        self.angular_y = msg.angular_velocity.y
        self.angular_z = msg.angular_velocity.z

        threshold_x = msg.linear_acceleration.x < 9.81 or msg.linear_acceleration.x > 9.81
        threshold_y = msg.linear_acceleration.y < 9.81 or msg.linear_acceleration.y > 9.81
        threshold_z = msg.linear_acceleration.z < 9.81 or msg.linear_acceleration.z > 9.81

        if (threshold_x or threshold_y or threshold_z):
            self.acc_magnitude = math.sqrt(msg.linear_acceleration.x**2 + msg.linear_acceleration.y**2 + msg.linear_acceleration.z**2)
        else:
            self.acc_magnitude = 9.81

    def accel_callback(self, msg):
        # Extract linear acceleration from the AccelStamped message
        self.linear_x = msg.accel.linear.x
        self.linear_y = msg.accel.linear.y
        self.linear_z = msg.accel.linear.z

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
        new_date = datetime.datetime.today().strftime("%Y-%m-%d")

        # Check if the date has changed
        if new_date != self.current_date:
            self.current_date = new_date
            self.csv_file.close()
            self.output_csv_path = os.path.join(self.output_csv_dir, f'{self.current_date}.csv')
            self.csv_file = open(self.output_csv_path, mode='a', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['Timestamp', 'Linear x', 'Linear y', 'Linear z', 'Filtered x', 'Filtered y', 'Filtered z', 'Angular x', 'Angular y', 'Angular z'])
        
        # Write data to CSV if the machine is on
        if self.machine:
            self.csv_writer.writerow([self.current_time, self.linear_x, self.linear_y, self.linear_z, self.filtered_x, self.filtered_y, self.filtered_z, self.angular_x, self.angular_y, self.angular_z])
            self.csv_file.flush()
            os.fsync(self.csv_file.fileno())
            self.get_logger().info('Data written to CSV file!')
    
    def sync_data(self):
        # Synchronize data to the cloud
        sync('/home/guts/Documents/accel_data/', '/media/guts/SD/accel_data_sd', 'sync')
        self.get_logger().info('Data synchronized to the cloud!')

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
