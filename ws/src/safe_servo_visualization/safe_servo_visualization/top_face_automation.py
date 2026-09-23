"""Move/capture/SAM adapter. Never picks or commits policy/slot inventory."""
import copy
import json
import time
import numpy as np
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .top_face_motion import recorded_grasp_rpy


class TopFaceAutomation:
    def _init_automation(self):
        self.inspection = None
        self.inspection_result_pub = self.create_publisher(String, '/top_face_debug/inspection_result', 10)
        self.create_subscription(String, '/top_face_debug/inspection_request', self._inspection_request, 10)

    def _inspection_owners(self):
        request = getattr(self, 'inspection', None)
        if not request:
            return set()
        owner = request['owner']
        status = self.motion_status.get(owner, {})
        expected = 'REARRANGE_INSPECT' if owner == 'policy' else 'SAM_INSPECTION'
        if (status.get('state') != expected or status.get('sam_request_id') != request['request_id'] or
                time.monotonic()-self.motion_seen.get(owner, 0) > 2):
            raise ValueError('inspection owner stopped, changed token, or disconnected')
        owners = {owner}
        if owner == 'slots':
            policy = self.motion_status.get('policy', {})
            if policy.get('state') == 'REARRANGE_RETRIEVE' and time.monotonic()-self.motion_seen.get('policy', 0) < 2:
                owners.add('policy')
            test = self.motion_status.get('test', {})
            if test.get('state') == 'RETRIEVE_SLOT' and time.monotonic()-self.motion_seen.get('test', 0) < 2:
                owners.add('test')
        return owners

    def _inspection_reply(self, request, success, **data):
        payload = {k: request[k] for k in ('request_id', 'owner', 'target')}
        self.inspection_result_pub.publish(String(data=json.dumps(dict(payload, success=success, **data), allow_nan=False)))

    def _inspection_request(self, msg):
        request = None
        try:
            request = json.loads(msg.data)
            if (request.get('owner') not in ('policy', 'slots', 'test') or
                    not all(isinstance(request.get(k), str) and request[k] for k in ('request_id', 'target'))):
                return
            if request.get('action') == 'cancel':
                if self.inspection and request['request_id'] == self.inspection['request_id'] and request['owner'] == self.inspection['owner']:
                    self._motion_fault('inspection canceled by owner')
                    self.generation += 1
                    self.inspection = None
                return
            if request.get('action') != 'inspect':
                return
            if self.inspection or self.motion_phase or self.future:
                self._inspection_reply(request, False, error='top-face debugger is busy')
                return
            self.inspection = request
            self._inspection_owners()
            self.inspection_started = time.monotonic()
            self.inspection_step = 'moving'
            # Resolve grasp now: missing legacy metadata must fail before motion.
            key = request['target'].split(':')[-1]
            self.inspection_grasp = self.motion_status['scene'].get('placed_marker_grasp_orientations', {}).get(key)
            recorded_grasp_rpy(self.inspection_grasp, 0.)
            self._idle_checks()
            if self.motion_status['pickup'].get('state') == 'AWAITING_GRASP':
                client = self.motion_clients['retreat']
                if not client.service_is_ready():
                    raise ValueError('contact retreat service is unavailable')
                self.inspection_retreat_id = int(self.motion_status['pickup']['operation_id']) + 1
                self.inspection_retreat_future = client.call_async(Trigger.Request())
                self.inspection_step = 'contact_retreat'
                self.state, self.message = 'CONTACT_RETREAT', 'Leaving measured contact vertically before inspection'
            else:
                self._begin_motion('move_to', request['target'])
        except Exception as exc:
            if request and self.inspection is request:
                self.inspection = None
                self._inspection_reply(request, False, error=str(exc))

    def _automation_tick(self):
        request = self.inspection
        if not request:
            return
        try:
            self._inspection_owners()
            if time.monotonic()-self.inspection_started > 175:
                raise ValueError('SAM inspection timeout')
            if self.state in ('FAULT', 'ERROR', 'REJECTED'):
                raise ValueError(self.message)
            if self.inspection_step == 'contact_retreat':
                pickup = self.motion_status['pickup']
                if time.monotonic()-self.motion_seen.get('pickup', 0) > 2:
                    raise ValueError('lost pickup status during contact retreat')
                if pickup.get('state') == 'FAULT':
                    raise ValueError(pickup.get('fault') or 'contact retreat failed')
                future = self.inspection_retreat_future
                if not future.done():
                    if time.monotonic()-self.inspection_started > 10:
                        raise ValueError('contact retreat service response timeout')
                    return
                response = future.result()
                if response is None or not response.success:
                    raise ValueError('contact retreat rejected: ' + str(getattr(response, 'message', 'no response')))
                op_id = int(pickup.get('operation_id', -1))
                if op_id > self.inspection_retreat_id:
                    raise ValueError('contact retreat replaced by another operation')
                if op_id != self.inspection_retreat_id or pickup.get('state') != 'SUCCEEDED':
                    return
                if pickup.get('object_info_obtained'):
                    raise ValueError('contact information was not invalidated by retreat')
                self.inspection_step = 'moving'
                self._begin_motion('move_to', request['target'])
                return
            if self.inspection_step == 'moving' and self.motion_phase is None:
                self.inspection_step = 'settling'
                self.inspection_settle_until = time.monotonic()+.75
            elif self.inspection_step == 'settling' and time.monotonic() >= self.inspection_settle_until:
                self.generation += 1
                self._capture(request['target'])
                if 'prompt_points' not in self.snapshot['meta']:
                    raise ValueError(self.message)
                self.worker_generation = self.generation
                self.future = self.pool.submit(self._segment, copy.deepcopy(self.snapshot))
                self.inspection_step = 'segmenting'
                self.state, self.message = 'SEGMENTING', 'Policy SAM inspection'
            elif self.inspection_step == 'segmenting' and self.future is None:
                result = self.result
                if self.state != 'ESTIMATED' or not result or 'top_center_base_m' not in result:
                    raise ValueError('SAM did not produce a validated top face')
                box, size, _ = self._target_geometry(request['target'])
                meta = self.snapshot['meta']
                from rclpy.time import Time
                camera_now = self._matrix(self.base, meta['camera_frame'], Time().to_msg())
                if not np.allclose(camera_now, meta['base_from_camera'], atol=.002, rtol=0):
                    raise ValueError('camera moved during SAM inference; capture again')
                if not np.allclose(box, meta['base_from_box'], atol=.001, rtol=0) or not np.allclose(size, meta['size_m'], atol=.001, rtol=0):
                    raise ValueError('recorded target changed during SAM inspection')
                rpy = recorded_grasp_rpy(self.inspection_grasp, result['yaw_rad'])
                self._inspection_reply(request, True,
                    top_center_base_m=result['top_center_base_m'], yaw_rad=result['yaw_rad'],
                    grasp_rpy_rad=list(rpy), size_m=size, color_stamp=meta['color_stamp'],
                    fitted_size_xy_m=result.get('fitted_size_xy_m'),
                    duration_sec=time.monotonic()-self.inspection_started)
                self.inspection = None
                self.message = 'SAM inspection complete; caller owns pickup and inventory.'
        except Exception as exc:
            self._motion_fault(str(exc))
            self.generation += 1
            self.inspection = None
            self._inspection_reply(request, False, error=str(exc))
