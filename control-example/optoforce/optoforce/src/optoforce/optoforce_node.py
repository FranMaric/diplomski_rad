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
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

import sys
import logging
import optoforce
import serial # required to handle exceptions raised in the optoforce module
import rospy
from geometry_msgs.msg import WrenchStamped

class OptoforceNode(object):
    """
    ROS interface for Optoforce sensors
    """

    CALIBRATION_SAMPLES = 500

    def __init__(self):
        """
        Initialize OptoforceDriver object
        """
        port = rospy.get_param("~port", "/dev/ttyACM0")
        sensor_type = rospy.get_param("~type", "m-ch/3-axis")
        starting_index = rospy.get_param("~starting_index", 0)
        scaling_factors = rospy.get_param("~scale")

        # Initialize optoforce driver
        try:
            self._driver = optoforce.OptoforceDriver(port,
                                                     sensor_type,
                                                     scaling_factors,
                                                     starting_index)
        except serial.SerialException as e:
            rospy.logfatal("Cannot connect to the sensor " + port
                           + (e.message if e.message else ''))
            rospy.signal_shutdown("Serial connection failure")
            raise

        # Create and advertise publishers for each connected sensor
        self._publishers = []
        self._wrenches = []
        topic_basename = "optoforce_"
        for i in range(self._driver.nb_sensors()):
            self._publishers.append(rospy.Publisher(topic_basename +
                                                    str(starting_index + i),
                                                    WrenchStamped,
                                                    queue_size=100))
            wrench = WrenchStamped()
            wrench.header.frame_id = topic_basename + str(starting_index + i)
            self._wrenches.append(wrench)

        self._bias = None
        self._calibration_samples = []

    def config(self):
        """
        Set the sensor's configuration based on parameters.

        The options to configure are speed (frequency at which data is
        delivered), filter frequency and zeroing of the values. The related
        ros param names are ~speed, ~filter and ~zero.
        """
        speed = rospy.get_param("~speed", "100Hz")
        filter = rospy.get_param("~filter", "15Hz")
        zero = rospy.get_param("~zero", "false")

        self._driver.config(speed, filter, zero)

    def run(self):
        """
        Runs the read loop.

        It listens to the serial port and publishes all force data it receives.
        Collects CALIBRATION_SAMPLES readings at startup to compute a bias
        offset that is subtracted from every subsequent measurement.
        """
        rospy.loginfo("Starting to listen to the sensor")
        rospy.loginfo("Collecting %d samples for bias calibration...",
                      self.CALIBRATION_SAMPLES)
        while not rospy.is_shutdown():
            data = self._driver.read()
            if isinstance(data, optoforce.OptoforceData):
                if self._bias is None:
                    self._calibration_samples.append(data)
                    if len(self._calibration_samples) >= self.CALIBRATION_SAMPLES:
                        self._compute_bias()
                else:
                    self._publish(data)
            elif isinstance(data, optoforce.OptoforceSerialNumber):
                rospy.loginfo("The sensor's serial number is " + str(data))

    def _compute_bias(self):
        n = len(self._calibration_samples)
        nb_sensors = self._driver.nb_sensors()
        nb_axis = self._driver.nb_axis()
        self._bias = [
            [sum(s.force[i][j] for s in self._calibration_samples) / n
             for j in range(nb_axis)]
            for i in range(nb_sensors)
        ]
        for i in range(nb_sensors):
            axes = ["fx", "fy", "fz", "tx", "ty", "tz"]
            bias_str = "  ".join(
                "%s:%.4f" % (axes[j], self._bias[i][j]) for j in range(nb_axis)
            )
            rospy.loginfo("Sensor %d bias — %s", i, bias_str)
        rospy.loginfo("Bias calibration complete. Starting to publish.")
        self._calibration_samples = []

    def _publish(self, data):
        stamp = rospy.Time.now()
        bias = self._bias
        for i in range(self._driver.nb_sensors()):
            self._wrenches[i].header.stamp = stamp
            self._wrenches[i].wrench.force.x = data.force[i][0] - bias[i][0]
            self._wrenches[i].wrench.force.y = data.force[i][1] - bias[i][1]
            self._wrenches[i].wrench.force.z = data.force[i][2] - bias[i][2]
            if self._driver.nb_axis() == 6:
                self._wrenches[i].wrench.torque.x = data.force[i][3] - bias[i][3]
                self._wrenches[i].wrench.torque.y = data.force[i][4] - bias[i][4]
                self._wrenches[i].wrench.torque.z = data.force[i][5] - bias[i][5]
            self._publishers[i].publish(self._wrenches[i])

class ConnectPythonLoggingToROS(logging.Handler):
    """
    Class interfacing logs using Python's standard logging facility and ROS's
    way of logging. It is meant to be used as a Handler for the logging module.

    Source: https://gist.github.com/nzjrs/8712011
    """

    MAP = {
        logging.DEBUG:rospy.logdebug,
        logging.INFO:rospy.loginfo,
        logging.WARNING:rospy.logwarn,
        logging.ERROR:rospy.logerr,
        logging.CRITICAL:rospy.logfatal
    }

    def emit(self, record):
        try:
            self.MAP[record.levelno]("%s: %s" % (record.name, record.msg))
        except KeyError:
            rospy.logerr("unknown log level %s LOG: %s: %s" % (record.levelno, record.name, record.msg))

if __name__ == '__main__':
    rospy.init_node("optoforce")
    try:
        node = OptoforceNode()
    except Exception as e:
        rospy.logfatal("Caught exception: " + str(e))
    else:
        #reconnect logging calls which are children of this to the ros log system
        logging.getLogger('optoforce').addHandler(ConnectPythonLoggingToROS())
        #logs sent to children of trigger with a level >= this will be redirected to ROS
        logging.getLogger('optoforce').setLevel(logging.DEBUG)

        node.config()
        node.run()
