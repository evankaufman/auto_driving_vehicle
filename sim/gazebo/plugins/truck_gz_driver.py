import gz.transport13 as transport
import gz.msgs10.double_pb2 as double_pb2
import rclpy
from std_msgs.msg import Float64MultiArray


class TruckGzDriver:
    def __init__(self):
        self._model_name = "suv_vehicle"
        self._joint_names = [
            "front_left_wheel_joint",
            "front_right_wheel_joint",
            "rear_left_wheel_joint",
            "rear_right_wheel_joint",
        ]
        self._drive_cmd_topic = "/mpc_ctrl/drive_cmd"
        self._node = transport.Node()
        self._publishers = []
        self._steer_publishers = []
        self._msg = double_pb2.Double()
        self._steer_msg = double_pb2.Double()
        self._steer_angle = 0.0
        self._last_print_time = None
        self._ros_node = None
        self._ros_sub = None
        self._latest_cmd = None

    def configure(self, entity, sdf, ecm, event_mgr):
        for joint in self._joint_names:
            topic = f"/model/{self._model_name}/joint/{joint}/cmd_force"
            self._publishers.append(self._node.advertise(topic, double_pb2.Double))

        steer_topics = ["steer_left_cmd", "steer_right_cmd"]
        for topic in steer_topics:
            self._steer_publishers.append(self._node.advertise(topic, double_pb2.Double))

        if not rclpy.ok():
            rclpy.init(args=None)
        self._ros_node = rclpy.create_node("truck_gz_driver")
        self._ros_sub = self._ros_node.create_subscription(
            Float64MultiArray, self._drive_cmd_topic, self._on_drive_cmd, 10
        )

    def _on_drive_cmd(self, msg):
        if len(msg.data) < 6:
            return
        self._latest_cmd = list(msg.data)

    def pre_update(self, info, ecm):
        if info.paused:
            return
        if self._ros_node is not None:
            rclpy.spin_once(self._ros_node, timeout_sec=0.0)
        if self._latest_cmd is None:
            return

        torque_fl, torque_fr, torque_rl, torque_rr, steer_angle, steer_rate = self._latest_cmd[:6]
        torques = [torque_fl, torque_fr, torque_rl, torque_rr]
        for pub, torque in zip(self._publishers, torques):
            self._msg.data = float(torque)
            pub.publish(self._msg)

        dt = info.dt.total_seconds() if info.dt.total_seconds() > 0 else 0.0
        target = float(steer_angle)
        max_step = abs(float(steer_rate)) * dt
        if max_step > 0.0:
            if target > self._steer_angle + max_step:
                self._steer_angle += max_step
            elif target < self._steer_angle - max_step:
                self._steer_angle -= max_step
            else:
                self._steer_angle = target
        else:
            self._steer_angle = target

        self._steer_msg.data = self._steer_angle
        for pub in self._steer_publishers:
            pub.publish(self._steer_msg)

        if self._last_print_time is None or (info.sim_time - self._last_print_time).total_seconds() >= 1.0:
            steer_deg = self._steer_angle * 180.0 / 3.141592653589793
            print(
                "truck_gz_driver torques="
                f"[{torque_fl:.1f},{torque_fr:.1f},{torque_rl:.1f},{torque_rr:.1f}] "
                f"steer_deg={steer_deg:.1f}"
            )
            self._last_print_time = info.sim_time


def get_system():
    return TruckGzDriver()
