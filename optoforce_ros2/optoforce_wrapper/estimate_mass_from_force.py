import rclpy
from rclpy.node import Node
import pandas as pd
import numpy as np
from sklearn.linear_model import LinearRegression
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64  # Added for mass publishing

class MassEstimatorNode(Node):
    def __init__(self):
        super().__init__('mass_estimator_node')

        self.prediction_buffer = []
        self.buffer_size = 20
        
        # 1. Load Data and Train Model
        self.csv_path = '/root/ros2_ws/src/optoforce_ros2/optoforce_wrapper/data/captured_values.csv'
        self.model = self._train_model()
        
        # 2. Setup ROS 2 Communication
        # Publisher for the estimated mass
        self.mass_publisher = self.create_publisher(Float64, 'mass_estimation', 10)

        # Subscribing to your filtered topic
        self.subscription = self.create_subscription(
            WrenchStamped,
            '/optoforce_0/wrench_filtered',
            self.listener_callback,
            10)
            
        self.get_logger().info('Mass Estimator Node Online. Model Trained.')

    def _train_model(self):
        try:
            df = pd.read_csv(self.csv_path)
            X = df[['fx', 'ty']] 
            y = df['target_weight']
            
            model = LinearRegression()
            model.fit(X, y)
            
            r_squared = model.score(X, y)
            self.get_logger().info(f'Model R^2 Score: {r_squared:.4f}')
            
            beta_1 = model.coef_[0]
            beta_2 = model.coef_[1]
            intercept = model.intercept_

            self.get_logger().info("--- TRAINED MODEL FUNCTION ---")
            self.get_logger().info(f"Mass = ({beta_1:.6f} * Fx) + ({beta_2:.6f} * Ty) + ({intercept:.6f})")
            self.get_logger().info("------------------------------")

            return model
            
        except Exception as e:
            self.get_logger().error(f'Failed to train model: {e}')
            return None

    def listener_callback(self, msg):
        if self.model is None:
            return

        # 1. Prepare data for prediction
        current_data = pd.DataFrame(
            [[msg.wrench.force.x, msg.wrench.torque.y]], 
            columns=['fx', 'ty']
        )
        
        # 2. Predict
        predicted_mass = self.model.predict(current_data)[0] - 0.019
        self.prediction_buffer.append(predicted_mass)

        # 3. If buffer is full, calculate average and publish
        if len(self.prediction_buffer) >= self.buffer_size:
            avg_mass = sum(self.prediction_buffer) / len(self.prediction_buffer)
            
            # Simple Deadband/Sanity check
            if avg_mass < 0.0:
                avg_mass = 0.0

            # 4. Publish the Result
            mass_msg = Float64()
            mass_msg.data = float(avg_mass)
            self.mass_publisher.publish(mass_msg)

            # Log the output
            self.get_logger().info(f"Published Mass: {avg_mass:.4f} kg")
            
            # 5. Reset buffer
            self.prediction_buffer = []

def main(args=None):
    rclpy.init(args=args)
    node = MassEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()