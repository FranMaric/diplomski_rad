#!/usr/bin/env python3
import rospy
import dynamic_reconfigure.client

rospy.init_node('set_compliance_params')

client = dynamic_reconfigure.client.Client(
    '/cartesian_impedance_example_controller/dynamic_reconfigure_compliance_param_node',
    timeout=30
)
client.update_configuration({
    'translational_stiffness': 400.0,
    'rotational_stiffness': 30.0,
    'nullspace_stiffness': 100.0,
})
