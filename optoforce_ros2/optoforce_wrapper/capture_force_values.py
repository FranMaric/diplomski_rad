import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from rclpy.qos import qos_profile_sensor_data

import os
import csv
import time

class FullWrenchFilterNode(Node):
    def __init__(self):
        super().__init__('wrench_filter_node')
        
        self.count = 0
        self.count_limit = 100
        self.payload_weight = 0.003
        
        # Dictionary to store the 'previous' filtered state for all 6 axes
        # Keys: fx, fy, fz, tx, ty, tz
        self.wrench = {
            'fx': None, 'fy': None, 'fz': None,
            'tx': None, 'ty': None, 'tz': None
        }
        
        self.subscription = self.create_subscription(
            WrenchStamped,
            '/optoforce_0/wrench_filtered',
            self.listener_callback,
            qos_profile_sensor_data)
        
        self.get_logger().info('Listener started...')

    
    def save_calibration_data(self, target_weight):
        """
        Appends current filtered readings and the known weight to a CSV file.
        
        Args:
            target_weight (float): The actual mass (kg) or force (N) 
                                currently applied to the sensor.
        """
        filename = '/root/ros2_ws/src/optoforce_ros2/optoforce_wrapper/data/captured_values.csv'
        file_exists = os.path.isfile(filename)

        # We extract only the relevant axes for your specific math (Fx and Ty)
        # But we include a timestamp for data integrity
        try:
            with open(filename, mode='a', newline='') as csvfile:
                fieldnames = ['fx', 'ty', 'target_weight']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

                # Write header only if the file was just created
                if not file_exists:
                    writer.writeheader()

                # Write the data row
                writer.writerow({
                    'fx': self.wrench['fx'] / self.count_limit,
                    'ty': self.wrench['ty'] / self.count_limit,
                    'target_weight': target_weight
                })
                
            self.get_logger().info(f'Data saved to {filename}: Fx={self.wrench["fx"]:.3f}, Ty={self.wrench["ty"]:.3f}')
        
        except Exception as e:
            self.get_logger().error(f'Failed to write to CSV: {str(e)}')

    def wrench_sum(self, key, current_value):
        if self.wrench[key] is None:
            self.wrench[key] = current_value
        else:
            self.wrench[key] += current_value
        return self.wrench[key]

    def listener_callback(self, msg):
        if self.count < self.count_limit:
            self.wrench_sum('fx', msg.wrench.force.x)
            self.wrench_sum('fy', msg.wrench.force.y)
            self.wrench_sum('fz', msg.wrench.force.z)
            self.wrench_sum('tx', msg.wrench.torque.x)
            self.wrench_sum('ty', msg.wrench.torque.y)
            self.wrench_sum('tz', msg.wrench.torque.z)

            self.count += 1
            self.get_logger().info(str(self.count) + '/' + str(self.count_limit))
        
        else:
            self.save_calibration_data(self.payload_weight)
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = FullWrenchFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()