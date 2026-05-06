import rclpy
from rclpy.node import Node
import pandas as pd
import numpy as np
from sklearn.linear_model import LinearRegression
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64  # Added for mass publishing

class ForceEstimatorNode(Node):
    def __init__(self):
        super().__init__('force_estimator_node')

        self.prediction_force_buffer = []
        self.buffer_size = 20
        
        # 1. Load Data and Train Model
        self.csv_path = '/home/rf-blue/Desktop/fran_ws/optoforce_ros2/data/captured_values.csv'
        self.model = self._train_model()
        if self.model is None:
            self.get_logger().error('Model training failed. Force estimation will not work.')
            return

        # 2. Setup ROS 2 Communication
        # Publisher for the estimated force
        self.force_publisher = self.create_publisher(Float64, 'force_estimation', 10)

        # Subscribing to your filtered topic
        self.subscription = self.create_subscription(
            WrenchStamped,
            '/optoforce_0/wrench_filtered',
            self.listener_callback,
            10)
            
        self.get_logger().info('Force Estimator Node Online. Model Trained.')

    def _train_model(self):
        try:
            df = pd.read_csv(self.csv_path)
            df.columns = df.columns.str.strip()
            X = df[['fz']]
            y = df['target_weight']
            
            model = LinearRegression()
            model.fit(X, y)
            
            r_squared = model.score(X, y)
            self.get_logger().info(f'Model R^2 Score: {r_squared:.4f}')
            
            beta_1 = model.coef_[0]
            intercept = model.intercept_

            self.get_logger().info("--- TRAINED MODEL FUNCTION ---")
            self.get_logger().info(f"Force = ({beta_1:.6f} * Fz) + ({intercept:.6f})")
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
            [[msg.wrench.force.z]], 
            columns=['fz']
        )
        
        # 2. Predict
        predicted_weight = self.model.predict(current_data)[0]
        predicted_force = predicted_weight * 9.81  # Convert mass to force (N)
        self.prediction_force_buffer.append(predicted_force)

        # 3. If buffer is full, calculate average and publish
        if len(self.prediction_force_buffer) >= self.buffer_size:
            avg_force = sum(self.prediction_force_buffer) / len(self.prediction_force_buffer)
            
            # Simple Deadband/Sanity check
            if avg_force < 0.0:
                avg_force = 0.0

            # 4. Publish the Result
            force_msg = Float64()
            force_msg.data = float(avg_force)
            self.force_publisher.publish(force_msg)

            # Log the output
            # self.get_logger().info(f"Published Force: {avg_force:.4f} N")
            
            # 5. Reset buffer
            self.prediction_force_buffer = []

def main(args=None):
    rclpy.init(args=args)
    node = ForceEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()