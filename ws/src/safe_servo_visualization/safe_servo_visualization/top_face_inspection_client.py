"""Token-bound inspection requests; inventory and grasp remain with the caller."""
import json
import time
import uuid
import math
from std_msgs.msg import String


def inspection_enabled(config):
    enabled = config.get('top_face_inspection_enabled', False)
    if not isinstance(enabled, bool):
        raise ValueError('top_face_inspection_enabled must be a YAML boolean')
    return enabled


def marker_target(scene, obstacle_id):
    matches = [key for key, value in scene.get('placed_marker_object_ids', {}).items()
               if value == obstacle_id]
    if len(matches) != 1:
        raise ValueError(f'no unique top-face marker for {obstacle_id!r}')
    return f'placed:placed_item_visuals:{matches[0]}'


def checked_result(payload):
    for key, count in (('top_center_base_m', 3), ('grasp_rpy_rad', 3), ('size_m', 3)):
        value = payload.get(key)
        if not isinstance(value, list) or len(value) != count or not all(math.isfinite(float(v)) for v in value):
            raise ValueError('invalid SAM inspection result: '+key)
    if min(payload['size_m']) <= 0 or not math.isfinite(float(payload['yaw_rad'])):
        raise ValueError('invalid SAM inspection geometry')
    return payload


class TopFaceInspectionClient:
    def __init__(self, node, owner):
        self.owner = owner
        self.token = None
        self.result = None
        self.pub = node.create_publisher(String, '/top_face_debug/inspection_request', 10)
        node.create_subscription(String, '/top_face_debug/inspection_result', self._result, 10)

    def begin(self, target):
        if self.token:
            raise ValueError('SAM inspection already active')
        self.token = uuid.uuid4().hex
        self.target = target
        self.started = time.monotonic()
        self.sent = False
        self.result = None

    def _result(self, msg):
        try:
            payload = json.loads(msg.data)
            if (self.token and payload.get('request_id') == self.token and
                    payload.get('owner') == self.owner and payload.get('target') == self.target):
                self.result = payload
        except (ValueError, TypeError):
            pass

    def tick(self):
        if not self.token:
            return None
        if time.monotonic()-self.started > 180:
            self.cancel()
            raise ValueError('top-face inspection timed out')
        # Let the caller's status announce this token before issuing motion.
        if not self.sent and time.monotonic()-self.started > .6:
            self.pub.publish(String(data=json.dumps(dict(action='inspect', owner=self.owner,
                request_id=self.token, target=self.target))))
            self.sent = True
        if self.result is None:
            return None
        result = self.result
        self.token = None
        if not result.get('success'):
            raise ValueError(result.get('error', 'SAM inspection failed'))
        return checked_result(result)

    def cancel(self):
        if self.token:
            self.pub.publish(String(data=json.dumps(dict(action='cancel', owner=self.owner,
                request_id=self.token, target=self.target))))
        self.token = None
        self.result = None
