#!/usr/bin/env python3
import rospy
from franka_msgs.srv import SetLoad, SetLoadRequest, SetEEFrame, SetEEFrameRequest

MASS    = 0.5                         # kg
COM     = [0.06717, -0.06717, 0.06]   # CoM from panda_link8 (flange) [m]
INERTIA = [0.001, 0, 0,
           0, 0.001, 0,
           0, 0, 0.0005]              # 3x3 column-major [kg*m^2]

# 4x4 column-major transform: panda_link8 (flange) → tcp
# Derived from static_transform_publisher args:
#   pos=[0.13435, -0.13435, 0.113], quat=[qx=0.382683, qy=-0.923880, qz=0, qw=0]
# Rotation matrix:
#   [[-0.707107, -0.707107,  0],
#    [-0.707107,  0.707107,  0],
#    [ 0,         0,        -1]]
NE_T_EE = [
    -0.707107, -0.707107,  0.0, 0.0,   # col 0
    -0.707107,  0.707107,  0.0, 0.0,   # col 1
     0.0,       0.0,      -1.0, 0.0,   # col 2
     0.13435,  -0.13435,   0.113, 1.0, # col 3 (translation)
]

LOAD_SERVICE = "/franka_control/set_load"
EE_SERVICE   = "/franka_control/set_EE_frame"


def main():
    rospy.init_node("set_ee_load", anonymous=False)
    rospy.sleep(0.5)  # let hardware interface finish advertising services

    try:
        rospy.loginfo("Waiting for %s ...", LOAD_SERVICE)
        rospy.wait_for_service(LOAD_SERVICE)
        proxy = rospy.ServiceProxy(LOAD_SERVICE, SetLoad)
        req = SetLoadRequest()
        req.mass = MASS
        req.F_x_center_load = COM
        req.load_inertia = INERTIA
        resp = proxy(req)
        if resp.success:
            rospy.loginfo("SetLoad OK — mass=%.3f kg, CoM=[%.4f, %.4f, %.4f] m", MASS, *COM)
        else:
            rospy.logerr("SetLoad FAILED: %s", resp.error)
    except rospy.ServiceException as e:
        rospy.logerr("SetLoad service call failed: %s", e)

    try:
        rospy.loginfo("Waiting for %s ...", EE_SERVICE)
        rospy.wait_for_service(EE_SERVICE)
        ee_proxy = rospy.ServiceProxy(EE_SERVICE, SetEEFrame)
        ee_req = SetEEFrameRequest()
        ee_req.NE_T_EE = NE_T_EE
        ee_resp = ee_proxy(ee_req)
        if ee_resp.success:
            rospy.loginfo("SetEEFrame OK — controller now tracks TCP frame")
        else:
            rospy.logerr("SetEEFrame FAILED: %s", ee_resp.error)
    except rospy.ServiceException as e:
        rospy.logerr("SetEEFrame service call failed: %s", e)


if __name__ == "__main__":
    main()
