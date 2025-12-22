import gz.transport13 as transport
import gz.msgs10.double_pb2 as double_pb2


class TruckGzDriver:
    def __init__(self):
        self._torque = 100.0
        self._model_name = "suv_vehicle"
        self._joint_names = [
            "front_left_wheel_joint",
            "front_right_wheel_joint",
            "rear_left_wheel_joint",
            "rear_right_wheel_joint",
        ]
        self._node = transport.Node()
        self._publishers = []
        self._steer_publishers = []
        self._msg = double_pb2.Double()
        self._steer_msg = double_pb2.Double()
        self._steer_angle = 0.0
        self._start_time = None
        self._steer_max_rad = 10.0 * 3.141592653589793 / 180.0
        self._steer_rate = 10.0 * 3.141592653589793 / 180.0
        self._last_print_time = None

    def configure(self, entity, sdf, ecm, event_mgr):
        for joint in self._joint_names:
            topic = f"/model/{self._model_name}/joint/{joint}/cmd_force"
            self._publishers.append(self._node.advertise(topic, double_pb2.Double))

        steer_topics = ["steer_left_cmd", "steer_right_cmd"]
        for topic in steer_topics:
            self._steer_publishers.append(self._node.advertise(topic, double_pb2.Double))

    def pre_update(self, info, ecm):
        if info.paused:
            return
        if self._start_time is None:
            self._start_time = info.sim_time
        elapsed = (info.sim_time - self._start_time).total_seconds()
        if elapsed < 3.0:
            self._msg.data = 0.0
        elif elapsed < 13.0:
            ramp = (elapsed - 3.0) / 10.0
            self._msg.data = self._torque * ramp
        else:
            self._msg.data = self._torque
        for pub in self._publishers:
            pub.publish(self._msg)

        # Steering command: 0 to 10 degrees with rate limiting.
        t_sec = (info.sim_time - self._start_time).total_seconds()
        target = 0.5 * (1.0 + __import__('math').sin(2.0 * __import__('math').pi * t_sec / 10.0)) * self._steer_max_rad
        dt = info.dt.total_seconds() if info.dt.total_seconds() > 0 else 0.0
        max_step = self._steer_rate * dt
        if target > self._steer_angle + max_step:
            self._steer_angle += max_step
        elif target < self._steer_angle - max_step:
            self._steer_angle -= max_step
        else:
            self._steer_angle = target
        self._steer_msg.data = self._steer_angle
        for pub in self._steer_publishers:
            pub.publish(self._steer_msg)

        if self._last_print_time is None or (info.sim_time - self._last_print_time).total_seconds() >= 1.0:
            print(f"truck_gz_driver torque={self._msg.data:.1f} Nm steer_deg={self._steer_angle * 180.0 / 3.141592653589793:.1f}")
            self._last_print_time = info.sim_time


def get_system():
    return TruckGzDriver()
