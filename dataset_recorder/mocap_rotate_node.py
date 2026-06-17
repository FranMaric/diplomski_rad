import threading
import tkinter as tk
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation

MOCAP_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

TOPICS = [
    '/vrpn_mocap/Brusilica/pose',
    '/vrpn_mocap/Kalup/pose',
]


class RotationState:
    def __init__(self, rx=90.0, ry=0.0, rz=0.0):
        self.rx = rx
        self.ry = ry
        self.rz = rz

    def get_rotation(self):
        # Extrinsic XYZ: rotate around world X first, then world Y, then world Z
        return Rotation.from_euler('xyz', [self.rx, self.ry, self.rz], degrees=True)


class MocapPoseRotator(Node):
    def __init__(self, state: RotationState):
        super().__init__('mocap_pose_rotator')
        self.state = state
        for topic_name in TOPICS:
            out_topic = topic_name.replace('/vrpn_mocap/', '/vrpn_mocap_rotated/', 1)
            pub = self.create_publisher(PoseStamped, out_topic, 10)
            self.create_subscription(
                PoseStamped, topic_name,
                lambda msg, p=pub: self._cb(msg, p),
                MOCAP_QOS)
            self.get_logger().info(f'Rotating {topic_name} -> {out_topic}')

    def _cb(self, msg, pub):
        rot = self.state.get_rotation()
        out = PoseStamped()
        out.header = msg.header

        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        new_pos = rot.apply(pos)
        out.pose.position.x = float(new_pos[0])
        out.pose.position.y = float(new_pos[1])
        out.pose.position.z = float(new_pos[2])

        q_old = [msg.pose.orientation.x, msg.pose.orientation.y,
                 msg.pose.orientation.z, msg.pose.orientation.w]
        if np.linalg.norm(q_old) < 1e-6:
            return
        q_new = (rot * Rotation.from_quat(q_old)).as_quat()
        out.pose.orientation.x = float(q_new[0])
        out.pose.orientation.y = float(q_new[1])
        out.pose.orientation.z = float(q_new[2])
        out.pose.orientation.w = float(q_new[3])

        pub.publish(out)


def build_gui(state: RotationState):
    root = tk.Tk()
    root.title('Mocap World-Frame Rotation')
    root.resizable(False, False)

    def make_slider(row, label, attr, init):
        tk.Label(root, text=label, width=6, anchor='e').grid(row=row, column=0, padx=(12, 4), pady=8)
        val_var = tk.StringVar(value=f'{init:+.1f}°')
        val_label = tk.Label(root, textvariable=val_var, width=7, anchor='w')
        val_label.grid(row=row, column=2, padx=(4, 12))

        def on_change(v):
            deg = float(v)
            setattr(state, attr, deg)
            val_var.set(f'{deg:+.1f}°')

        slider = tk.Scale(
            root, from_=-180, to=180, resolution=0.5,
            orient=tk.HORIZONTAL, length=320,
            showvalue=False, command=on_change,
        )
        slider.set(init)
        slider.grid(row=row, column=1, pady=8)

    make_slider(0, 'X', 'rx', state.rx)
    make_slider(1, 'Y', 'ry', state.ry)
    make_slider(2, 'Z', 'rz', state.rz)

    root.mainloop()


def main():
    state = RotationState(rx=90.0, ry=0.0, rz=0.0)

    rclpy.init()
    node = MocapPoseRotator(state)

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    build_gui(state)  # blocks until window is closed

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
