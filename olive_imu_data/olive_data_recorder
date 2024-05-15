import rclpy
import math
import csv
import datetime
from rclpy.node import Node
import rclpy.timer
from sensor_msgs.msg import Imu, MagneticField, Temperature
from geometry_msgs.msg import AccelStamped

class RecNode(Node):

    def __init__(self):
        super().__init__('rec_node')

        # Open CSV file for writing
        self.csv_file = open('IMU_data.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write header
        self.csv_writer.writerow(['Timestamp', 'IMU Acceleration', 'Magnetic field', 'Temperature', 'Linear Acceleration', 'Angular Acceleration'])

        # Create subscription to Accel data
        self.subscription = self.create_subscription(
            AccelStamped,
            '/olive/imu/id01/linear_accel',
            self.accel_callback,
            10)
        
        # Create subscription to Temp data
        self.subscription_temp= self.create_subscription(
            Temperature,
            '/olive/imu/id01/temperature',
            self.temp_callback,
            10)
        
        # Create subscription to IMU data
        self.subscription_Imu = self.create_subscription(
            Imu,
            '/olive/imu/id01/filtered_imu',
            self.imu_callback,
            10)
        
        # Create subscription to Magnetometer data
        self.subscription = self.create_subscription(
            MagneticField,
            '/olive/imu/id01/magnetometer',
            self.magnetic_callback,
            10)
        
        # Create timers to perform the operation every "x" seconds
        self.timer1 = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('I am Batman!')

    def imu_callback(self, msg):
        
        # Calculate acceleration magnitude
        self.acc_magnitude = math.sqrt(msg.linear_acceleration.x**2 +msg.linear_acceleration.y**2 + msg.linear_acceleration.z**2 )

    def magnetic_callback(self, msg):

        # Calculate magnetic field magnitude
        self.mag_field = math.sqrt(msg.magnetic_field.x**2 + msg.magnetic_field.y**2 + msg.magnetic_field.z**2 )

    def temp_callback(self, msg):

        # Calculate temperature
        self.temp = msg.temperature

    def accel_callback(self, msg):

        # Calculate linear acceleration magnitude
        self.linear_acc = math.sqrt(msg.accel.linear.x**2 + msg.accel.linear.y**2 + msg.accel.linear.z**2)

        # Calculate angular acceleration magnitude
        self.angular_acc = math.sqrt(msg.accel.angular.x**2 + msg.accel.angular.y**2 + msg.accel.angular.z**2)

    def timer_callback(self):

        # Set the actual time 
        self.current_time = datetime.datetime.today().replace(microsecond=0)
        self.csv_writer.writerow([self.current_time, self.acc_magnitude, self.mag_field, self.temp, self.linear_acc, self.angular_acc])
        self.get_logger().info('Timestamp: %s, IMU Acceleration: %.2f m/s², Magnetic field: %2f Tesla, Temperature: %2f degC, Linear Acceleration: %2f m/s², Angular Acceleration: %2f rad/s²' % (self.current_time, self.acc_magnitude, self.mag_field, self.temp, self.linear_acc, self.angular_acc))
    
    def destroy_node(self):
        # Close CSV file when node is shutting down
        self.csv_file.close()
        super().destroy_node()
        
        
def main(args=None):
    rclpy.init(args=args)
    rec_node = RecNode()    
    rclpy.spin(rec_node)
    rec_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
