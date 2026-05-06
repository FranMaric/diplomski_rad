import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from rclpy.qos import qos_profile_sensor_data

class FullWrenchFilterNode(Node):
    def __init__(self):
        super().__init__('wrench_filter_node')
        
        # Filter strength (0.01 is strong/smooth)
        self.alpha = 0.07
        
        # Dictionary to store the 'previous' filtered state for all 6 axes
        # Keys: fx, fy, fz, tx, ty, tz
        self.filtered_state = {
            'fx': None, 'fy': None, 'fz': None,
            'tx': None, 'ty': None, 'tz': None
        }
        
        self.subscription = self.create_subscription(
            WrenchStamped,
            '/optoforce_0',
            self.listener_callback,
            qos_profile_sensor_data) 
            
        self.publisher_ = self.create_publisher(
            WrenchStamped, 
            '/optoforce_0/wrench_filtered', 
            10)
        
        self.get_logger().info('6-DOF Filter Node Started (Alpha: 0.01)')

    def filter_value(self, key, current_value):
        """Helper to apply EMA to a specific axis"""
        if self.filtered_state[key] is None:
            self.filtered_state[key] = current_value
        else:
            self.filtered_state[key] = (self.alpha * current_value) + \
                                       ((1.0 - self.alpha) * self.filtered_state[key])
        return self.filtered_state[key]

    def listener_callback(self, msg):
        # Create a new WrenchStamped message for the output
        filtered_msg = WrenchStamped()
        
        # Copy the header (keeps the timestamp and frame_id consistent)
        filtered_msg.header = msg.header
        
        # Apply filter to Forces
        # filtered_msg.wrench.force.x = self.filter_value('fx', msg.wrench.force.x)
        # filtered_msg.wrench.force.y = self.filter_value('fy', msg.wrench.force.y)
        filtered_msg.wrench.force.z = self.filter_value('fz', msg.wrench.force.z)
        
        # Apply filter to Torques
        # filtered_msg.wrench.torque.x = self.filter_value('tx', msg.wrench.torque.x)
        # filtered_msg.wrench.torque.y = self.filter_value('ty', msg.wrench.torque.y)
        # filtered_msg.wrench.torque.z = self.filter_value('tz', msg.wrench.torque.z)
        
        # Publish the full 6-DOF filtered wrench
        self.publisher_.publish(filtered_msg)

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