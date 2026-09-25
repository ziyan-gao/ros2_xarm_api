"""Validated task-owned settings accepted by the shared motion executor.

No task selection or motion is performed here. The task sends one immutable
policy before starting; the executor acknowledges its token in status.
"""
import json
import math
import time
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String

FIELDS = dict(pick_force_n=('force_threshold', 0., 100.),
              transport_force_n=('transport_force_threshold', 0., 100.),
              place_force_n=('place_force_threshold', 0., 100.),
              departure_lift_m=('transport_departure_lift_m', 0., .100),
              retreat_speed_mm_s=('return_clearance_speed', 0., 30.))
FLAGS = ('sdk_pick_retreat', 'sdk_place_retreat')


def validated_policy(value):
    data = json.loads(value)
    if not isinstance(data, dict) or set(data) != {'task', 'token', 'motion'}:
        raise ValueError('task policy requires task, token and motion')
    if data['task'] not in ('pack_new', 'unpack', 'pack_slot', 'repack'):
        raise ValueError('unknown task')
    if not isinstance(data['token'], str) or not 1 <= len(data['token']) <= 128:
        raise ValueError('invalid task token')
    motion = data['motion']
    if not isinstance(motion, dict) or set(motion) != set(FIELDS) | set(FLAGS):
        raise ValueError('incomplete/unknown task motion settings')
    for key, (_, low, high) in FIELDS.items():
        v = motion[key]
        if type(v) not in (float, int) or not math.isfinite(v) or not low < v <= high:
            raise ValueError(f'{key} must be finite in ({low}, {high}]')
    for key in FLAGS:
        if type(motion[key]) is not bool:
            raise ValueError(key+' must be boolean')
    return data


class TaskMotionPolicy:
    def _init_task_motion_policy(self):
        self.declare_parameter('task_motion_policy', '')
        self.task_policy_token = None
        self.task_policy_owner = None
        self.task_policy_seen = 0.
        self.task_policy_active = False
        self.task_policy_defaults = {attr: getattr(self, attr) for attr, _, _ in FIELDS.values()}
        self.task_policy_defaults.update(sdk_pick_retreat=True, sdk_place_retreat=True)
        self.sdk_pick_retreat = self.sdk_place_retreat = True
        self.add_on_set_parameters_callback(self._task_policy_parameters)
        self.create_subscription(String, '/pick_place_test/status', self._task_policy_session, 10)
        self.create_timer(.2, self._task_policy_tick)

    def _task_policy_session(self, msg):
        try:
            data = json.loads(msg.data)
            self.task_policy_owner = data.get('task_policy_token')
            self.task_policy_seen = time.monotonic()
        except (ValueError, TypeError, AttributeError):
            pass

    def _task_policy_parameters(self, parameters):
        selected = [p for p in parameters if p.name == 'task_motion_policy']
        if not selected:
            return SetParametersResult(successful=True)
        try:
            if len(parameters) != 1:
                raise ValueError('set task policy separately')
            data = validated_policy(selected[0].value)
            if self.state in self.ACTIVE:
                raise ValueError('motion executor is active; policy cannot change')
            if (self.task_policy_owner != data['token'] or
                    time.monotonic()-self.task_policy_seen > 3.):
                raise ValueError('waiting for task session ownership acknowledgement')
            for key, (attr, _, _) in FIELDS.items():
                setattr(self, attr, float(data['motion'][key]))
            for key in FLAGS:
                setattr(self, key, data['motion'][key])
            self.task_policy_task = data['task']
            self.task_policy_token = data['token']
            self.task_policy_active = True
            self.get_logger().info('accepted task policy '+json.dumps(data, sort_keys=True))
            return SetParametersResult(successful=True)
        except (ValueError, TypeError, KeyError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))

    def _task_policy_tick(self):
        if not self.task_policy_active or self.state in self.ACTIVE:
            return
        if self.task_policy_owner == self.task_policy_token and time.monotonic()-self.task_policy_seen < 3.:
            return
        for attr, value in self.task_policy_defaults.items():
            setattr(self, attr, value)
        self.task_policy_token = None
        self.task_policy_active = False
