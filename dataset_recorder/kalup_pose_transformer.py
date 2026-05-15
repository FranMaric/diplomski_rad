import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import math

MOCAP_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

# transofrmira iz tezista trokuta do centra cepa boce
# transform prema cadu
TRANSLATION = [-(0.0277 - 0.006 + 0.03505 + 0.0128),
               0.011 + 0.17905,
               -(0.00925 + 0.076 + 0.01415)]  # [x, y, z] in meters

TRANSLATION = [ # dodavanje eksperimentalnih vrijednosti koje rade :shrug:
    TRANSLATION[0] - 0.005,
    TRANSLATION[1] + 0.01,
    TRANSLATION[2] + 0.022,
]

class KalupPoseTransformer(Node):
    def __init__(self):
        super().__init__('kalup_pose_transformer')
        self.sub = self.create_subscription(
            PoseStamped, '/vrpn_mocap/Kalup/pose', self.cb, MOCAP_QOS)
        self.pub = self.create_publisher(
            PoseStamped, '/kalup_cep', 10)
        print("KalupPoseTransformer initialized.")

    def cb(self, msg):
        out = PoseStamped()
        out.header = msg.header
        out.pose.position.x = msg.pose.position.x + TRANSLATION[0]
        out.pose.position.y = msg.pose.position.y + TRANSLATION[1]
        out.pose.position.z = msg.pose.position.z + TRANSLATION[2]
        out.pose.orientation = msg.pose.orientation
        self.pub.publish(out)

def euclidean_distance(a, b) -> float:
    return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2)


# koristi se za tuniranje transform vrijednosti
class DistanceDeltaPublisher(Node):
    def __init__(self):
        super().__init__('distance_delta_publisher')

        self.kalup_position = None
        self.create_subscription(PoseStamped, '/vrpn_mocap/Brusilica/pose', self.cb_brusilica, MOCAP_QOS)
        self.create_subscription(PoseStamped, '/kalup_cep', self.cb_kalup_cep, MOCAP_QOS)
        self.pub_x = self.create_publisher(Float32, '/delta_brusilica_kalup_cep/x', 10)
        self.pub_y = self.create_publisher(Float32, '/delta_brusilica_kalup_cep/y', 10)
        self.pub_z = self.create_publisher(Float32, '/delta_brusilica_kalup_cep/z', 10)
        self.pub_xyz = self.create_publisher(Float32, '/delta_brusilica_kalup_cep/euclidean_dist', 10)


        print("DistanceDeltaPublisher initialized.")

    def cb_kalup_cep(self, msg):
        self.kalup_position = msg.pose.position

    def cb_brusilica(self, msg):
        if self.kalup_position is None:
            return
        p = msg.pose.position
        k = self.kalup_position
        msg_x, msg_y, msg_z = Float32(), Float32(), Float32()
        msg_x.data = p.x - k.x
        msg_y.data = p.y - k.y
        msg_z.data = p.z - k.z
        self.pub_x.publish(msg_x)
        self.pub_y.publish(msg_y)
        self.pub_z.publish(msg_z)

        msg = Float32()
        msg.data = euclidean_distance(p, k)
        self.pub_xyz.publish(msg)

def main():
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(KalupPoseTransformer())
    # executor.add_node(DistanceDeltaPublisher())
    executor.spin()

main()