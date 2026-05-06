#!/usr/bin/env python3
"""
Kalibracija OptoForce / F/T senzora preko ROS 2 topica.
Subscriba se na WrenchStamped, skuplja N uzoraka, ispiše Mean/StdDev/Min/Max.

Pokretanje:
  ros2 run <tvoj_paket> read_current_avg_value
  # ili direktno:
  python3 read_current_avg_value.py --ros-args -p topic:=/optoforce_0 -p n_samples:=2000
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import WrenchStamped
import statistics


class WrenchCalibrator(Node):
    def __init__(self):
        super().__init__('wrench_calibrator')

        # Parametri (mogu se override-at preko --ros-args -p)
        self.declare_parameter('topic', '/optoforce_0')
        self.declare_parameter('n_samples', 1000)

        topic = self.get_parameter('topic').value
        self.n_samples = self.get_parameter('n_samples').value

        # Spremnici za uzorke
        self.samples = {
            'fx': [], 'fy': [], 'fz': [],
            'tx': [], 'ty': [], 'tz': []
        }
        self.done = False

        self.subscription = self.create_subscription(
            WrenchStamped,
            topic,
            self.callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            f'Slušam {topic}, skupljam {self.n_samples} uzoraka...')
        self.get_logger().info('Drži masu mirno na senzoru.')

    def callback(self, msg: WrenchStamped):
        if self.done:
            return

        self.samples['fx'].append(msg.wrench.force.x)
        self.samples['fy'].append(msg.wrench.force.y)
        self.samples['fz'].append(msg.wrench.force.z)
        self.samples['tx'].append(msg.wrench.torque.x)
        self.samples['ty'].append(msg.wrench.torque.y)
        self.samples['tz'].append(msg.wrench.torque.z)

        n = len(self.samples['fz'])
        if n % 100 == 0:
            self.get_logger().info(f'  {n}/{self.n_samples}')

        if n >= self.n_samples:
            self.done = True
            self.print_stats()
            # Ugasi node
            rclpy.shutdown()

    def print_stats(self):
        n = len(self.samples['fz'])
        print(f'\nGotovo. Rezultati ({n} uzoraka):\n')
        print(f"{'Os':<5} {'Mean':>12} {'StdDev':>10} {'Min':>10} {'Max':>10}")
        print('-' * 52)
        for key in ['fx', 'fy', 'fz', 'tx', 'ty', 'tz']:
            data = self.samples[key]
            mean = statistics.mean(data)
            stdev = statistics.stdev(data) if len(data) > 1 else 0.0
            print(f"{key:<5} {mean:>12.4f} {stdev:>10.4f} "
                  f"{min(data):>10.4f} {max(data):>10.4f}")
        print()


def main(args=None):
    rclpy.init(args=args)
    node = WrenchCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()