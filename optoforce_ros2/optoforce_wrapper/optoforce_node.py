#!/usr/bin/env python3

# Copyright 2015 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

import sys
import os
import logging
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import WrenchStamped
import serial  # required to handle exceptions raised in the optoforce module

from optoforce_wrapper import optoforce

class OptoforceNode(Node):
    """
    ROS2 interface for Optoforce sensors
    """

    def __init__(self):
        super().__init__('optoforce_node')
        self.get_logger().info('Initializing OptoforceNode...')

        # Declare parameters
        self.declare_parameter(
            'port',
            '/dev/ttyACM0',
            ParameterDescriptor(description='The serial port for the sensor.')
        )
        self.declare_parameter(
            'type',
            's-ch/6-axis',
            ParameterDescriptor(description='The type of the sensor.')
        )
        self.declare_parameter(
            'starting_index',
            0,
            ParameterDescriptor(description='The starting index for the sensor topics.')
        )
        self.declare_parameter(
            'scale',
            [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            ParameterDescriptor(description='Scaling factors for the sensor axes.')
        )
        self.declare_parameter(
            'speed',
            '100Hz',
            ParameterDescriptor(description='The data frequency (e.g., 100Hz).')
        )
        self.declare_parameter(
            'filter',
            '15Hz',
            ParameterDescriptor(description='The filter frequency (e.g., 15Hz).')
        )
        self.declare_parameter(
            'zero',
            True,
            ParameterDescriptor(description='Zero the sensor on start.')
        )

        # Get parameter values
        port = self.get_parameter('port').get_parameter_value().string_value
        sensor_type = self.get_parameter('type').get_parameter_value().string_value
        starting_index = self.get_parameter('starting_index').get_parameter_value().integer_value
        scaling_factors = self.get_parameter('scale').get_parameter_value().double_array_value
        speed = self.get_parameter('speed').get_parameter_value().string_value
        filter_freq = self.get_parameter('filter').get_parameter_value().string_value
        zero = self.get_parameter('zero').get_parameter_value().bool_value

        # Initialize optoforce driver
        try:
            self._driver = optoforce.OptoforceDriver(port,
                                                     sensor_type,
                                                     [scaling_factors],
                                                     starting_index)
        except serial.SerialException as e:
            self.get_logger().fatal(f"Cannot connect to the sensor on port {port}: {e}")
            sys.exit(1)

        # Create and advertise publishers for each connected sensor
        self._publishers = []
        self._wrenches = []
        topic_basename = "optoforce_"
        for i in range(self._driver.nb_sensors()):
            topic_name = topic_basename + str(starting_index + i)
            self._publishers.append(self.create_publisher(WrenchStamped, topic_name, 100))
            wrench = WrenchStamped()
            wrench.header.frame_id = topic_name
            self._wrenches.append(wrench)

        # Configure the sensor
        self.config(speed, filter_freq, zero)
        self.get_logger().info("Starting to listen to the sensor")

        publish_rate = 100
        self.create_timer(1.0 / publish_rate, self.run)

    def config(self, speed, filter_freq, zero):
        """
        Set the sensor's configuration based on parameters.
        """
        self._driver.config(speed, filter_freq, True)

    def run(self):
        """
        Runs the read loop.
        """
        try:
            data = self._driver.read()
            if isinstance(data, optoforce.OptoforceData):
                self._publish(data)
            elif isinstance(data, optoforce.OptoforceSerialNumber):
                self.get_logger().info(f"The sensor's serial number is {data}")
        except Exception as e:
            self.get_logger().error(f"Caught exception in read loop: {e}")

    def _publish(self, data):
        stamp = self.get_clock().now().to_msg()
        for i in range(self._driver.nb_sensors()):
            self._wrenches[i].header.stamp = stamp
            self._wrenches[i].wrench.force.x = float(data.force[i][0])
            self._wrenches[i].wrench.force.y = float(data.force[i][1])
            self._wrenches[i].wrench.force.z = float(data.force[i][2])
            if self._driver.nb_axis() == 6:
                self._wrenches[i].wrench.torque.x = float(data.force[i][3])
                self._wrenches[i].wrench.torque.y = float(data.force[i][4])
                self._wrenches[i].wrench.torque.z = float(data.force[i][5])
            self._publishers[i].publish(self._wrenches[i])

def main(args=None):
    rclpy.init(args=args)
    try:
        node = OptoforceNode()
        rclpy.spin(node)
    except Exception as e:
        get_logger('optoforce_node').fatal(f"Caught exception: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()