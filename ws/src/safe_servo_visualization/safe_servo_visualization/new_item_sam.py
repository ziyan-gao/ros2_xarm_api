"""New-item refinement before publishing a pre-grasp target."""
import copy
import math
import time

from visualization_msgs.msg import MarkerArray
from .top_face_inspection_client import TopFaceInspectionClient


def refined_marker(coarse, result):
    """Keep coarse Z/height, replace only planar geometry."""
    marker = copy.deepcopy(coarse)
    center = result['top_center_base_m']
    expected_top = coarse.pose.position.z + coarse.scale.z/2
    if abs(center[2]-expected_top) > .005:
        raise ValueError('SAM changed coarse top height')
    if math.hypot(center[0]-coarse.pose.position.x, center[1]-coarse.pose.position.y) > .05:
        raise ValueError('SAM center exceeds coarse-box correction limit')
    dims = result['size_m']
    if any(abs(a-b) > .025 for a, b in zip(dims[:2], (coarse.scale.x, coarse.scale.y))):
        raise ValueError('SAM dimensions exceed coarse-box correction limit')
    marker.pose.position.x, marker.pose.position.y = map(float, center[:2])
    marker.scale.x, marker.scale.y = map(float, dims[:2])
    yaw = float(result['yaw_rad'])
    marker.pose.orientation.x = marker.pose.orientation.y = 0.
    marker.pose.orientation.z, marker.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
    return marker


class NewItemSAM:
    def _init_new_item_sam(self):
        self.new_item_sam_active = False
        self.new_item_sam_message = ''
        self.new_item_sam_retry_at = 0.
        self.new_item_sam_client = TopFaceInspectionClient(self, 'new_item')
        self.sam_coarse_pub = self.create_publisher(MarkerArray, '/object_info_estimation/sam_coarse_boxes', 10)

    def estimate_object_info_sam_callback(self, request, response):
        if self.state in self.ACTIVE:
            response.message = 'pickup pipeline is already active'
            return response
        if self.pickup_status.get('state') == 'AWAITING_GRASP':
            response.message = 'leave/reconcile existing contact before a new SAM estimate'
            return response
        result = self.estimate_object_info_callback(request, response)
        if result.success:
            self.new_item_sam_active = True
        return result

    def _begin_new_item_sam(self, marker):
        self.sam_coarse_marker = copy.deepcopy(marker)
        self.sam_coarse_pub.publish(MarkerArray(markers=[marker]))
        self.new_item_sam_client.begin(f'coarse:{marker.ns}:{marker.id}')
        self.state = 'SAM_REFINEMENT'
        self.new_item_sam_message = 'SAM refinement before pre-grasp; no motion'
        self.publish_status()

    def _tick_new_item_sam(self):
        if self.motion_status.get('state') == 'FAULT' or self.pickup_status.get('state') == 'FAULT':
            self._set_fault('hardware/motion fault during new-item SAM refinement')
            return
        self.sam_coarse_pub.publish(MarkerArray(markers=[self.sam_coarse_marker]))
        try:
            result = self.new_item_sam_client.tick()
            if result is None:
                return
            # The client binds the result to this request/target and validates
            # geometry. The server checks camera and frozen target transforms.
            # Do not discard successful SAM because the live depth stream has
            # a missing, stale, or noisy box after the RGB capture.
            marker = refined_marker(self.sam_coarse_marker, result)
            marker.header.stamp = self.get_clock().now().to_msg()
            self.stable_box_pub.publish(MarkerArray(markers=[marker]))
            self.stable_box_published_at = time.monotonic()
            self.state = self.WAIT_DETECTION
            self.phase_started = time.monotonic()
            self.new_item_sam_message = 'SAM accepted; XY refined, Z/height unchanged'
            self.publish_status()
        except Exception as exc:
            self.new_item_sam_client.cancel()
            self._reset_detection_samples()
            self.state = self.WAIT_DETECTION
            self.phase_started = time.monotonic()
            self.new_item_sam_retry_at = time.monotonic()+2.
            self.new_item_sam_message = f'Waiting to retry SAM: {exc}'
            self.get_logger().warning(self.new_item_sam_message)
            self.publish_status()
