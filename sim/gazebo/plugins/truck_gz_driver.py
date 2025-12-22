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
        self._msg = double_pb2.Double()
        self._start_time = None
        self._last_print_time = None

    def configure(self, entity, sdf, ecm, event_mgr):
        for joint in self._joint_names:
            topic = f"/model/{self._model_name}/joint/{joint}/cmd_force"
            self._publishers.append(self._node.advertise(topic, double_pb2.Double))

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
        if self._last_print_time is None or (info.sim_time - self._last_print_time).total_seconds() >= 1.0:
            print(f"truck_gz_driver torque={self._msg.data:.1f} Nm")
            self._last_print_time = info.sim_time


def get_system():
    return TruckGzDriver()
