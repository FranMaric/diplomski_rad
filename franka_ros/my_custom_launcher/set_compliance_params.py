#!/usr/bin/env python3
import rospy
import dynamic_reconfigure.client

rospy.init_node('set_compliance_params')

client = dynamic_reconfigure.client.Client(
    '/cartesian_impedance_controller/dynamic_reconfigure_compliance_param_node',
    timeout=30
)
client.update_configuration({
    'translational_stiffness': 5000.0,
    'rotational_stiffness': 150.0,
})
