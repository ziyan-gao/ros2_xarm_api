"""Offline diagnostics tests: no hardware, motion, or SAM checkpoint required."""
import json
import math
import sys
from contextlib import nullcontext
from concurrent.futures import Future
from types import SimpleNamespace as NS

import numpy as np
import pytest
import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray

from safe_servo_visualization.top_face_geometry import (
    transform, top_points, project, prompts, fit_top)
from safe_servo_visualization.top_face_debug_node import TopFaceDebug, fresh_frame, image_array


def scene():
    k = np.array([[600., 0., 320.], [0., 600., 240.], [0., 0., 1.]])
    camera = transform([0, 0, .8], [1, 0, 0, 0])
    size = [.2, .12, .16]
    prior = top_points(size, transform([.01, 0, .12], [0, 0, 0, 1]))
    depth = np.full((480, 640), .6, np.float32)
    mask = np.zeros_like(depth, bool)
    mask[180:301, 220:421] = True
    return k, camera, size, prior, depth, mask


def test_project_known_top_and_prompts():
    k, camera, size, prior, _, _ = scene()
    pixels = project(prior, np.linalg.inv(camera), k, np.zeros(5))
    assert pixels[0] == pytest.approx([330, 240])
    assert prior[0] == pytest.approx([.01, 0, .2])
    points, box = prompts(pixels, 640, 480)
    assert points.shape == (5, 2)
    assert box == pytest.approx([222, 172, 438, 308])


def test_geometry_rejects_invalid_projection():
    with pytest.raises(ValueError, match='behind'):
        project([[0., 0., -.1]], np.eye(4), np.eye(3), np.zeros(5))
    with pytest.raises(ValueError, match='outside'):
        prompts(np.zeros((5, 2)), 640, 480)
    with pytest.raises(ValueError, match='zero quaternion'):
        transform([0, 0, 0], [0, 0, 0, 0])
    with pytest.raises(ValueError, match='positive'):
        top_points([.2, 0, .1], np.eye(4))


@pytest.mark.parametrize('yaw', [0., math.pi])
def test_fit_top_recovers_center_and_preserves_symmetric_yaw(yaw):
    k, camera, size, prior, depth, mask = scene()
    fit = fit_top(mask, k, np.zeros(5), camera, prior, size, yaw)
    assert fit['top_center_base_m'] == pytest.approx([0, 0, .2], abs=.001)
    assert fit['delta_center_m'] == pytest.approx([-.01, 0, 0], abs=.001)
    assert fit['yaw_rad'] == pytest.approx(yaw, abs=.01)
    assert fit['diagnostic_only']
    assert fit['uses_measured_depth'] is False
    assert fit['recorded_top_z_m'] == pytest.approx(.2)


def test_empty_mask_and_wrong_size_are_rejected():
    k, camera, size, prior, depth, mask = scene()
    with pytest.raises(ValueError, match='segmented pixels'):
        fit_top(np.zeros_like(mask), k, np.zeros(5), camera, prior, size, 0)
    with pytest.raises(ValueError, match='footprint'):
        fit_top(mask, k, np.zeros(5), camera, prior, [.4, .3, .16], 0)


def frame(t):
    return NS(header=NS(stamp=NS(sec=int(t), nanosec=round((t-int(t))*1e9))))


def test_rgb_selection_requires_no_depth():
    c1, c2, d1, d2 = frame(10.), frame(10.5), frame(10.02), frame(10.2)
    assert fresh_frame([c1, c2], 10.6) == c2
    with pytest.raises(ValueError, match='fresh RGB'):
        fresh_frame([c1], 12.)
    with pytest.raises(ValueError):
        fresh_frame([c2], 10.)


@pytest.fixture
def node():
    rclpy.init()
    instance = TopFaceDebug()
    yield instance
    instance.pool.shutdown(wait=True, cancel_futures=True)
    instance.destroy_node()
    rclpy.shutdown()


def test_node_starts_idle_and_no_low_level_motion_publishers(node):
    assert node.motion_phase is None
    assert node.motion_future is None
    topics = {pub.topic_name for pub in node.publishers}
    assert topics <= {'/top_face_debug/status', '/top_face_debug/preview', '/rosout', '/parameter_events',
                      '/top_face_debug/view_target', '/staging_slots/retrieve_target'}


def test_missing_sam_checkpoint_is_actionable(node):
    with pytest.raises(ValueError, match='sam_checkpoint'):
        node._segment({})


def capture_synthetic(node):
    k, camera, size, prior, depth, mask = scene()
    now = node.get_clock().now().to_msg()
    rgb = Image(height=480, width=640, encoding='rgb8', step=640*3,
                data=np.zeros((480, 640, 3), np.uint8).tobytes())
    rgb.header.stamp = now
    rgb.header.frame_id = 'camera_color_optical_frame'
    info = CameraInfo()
    info.header = rgb.header
    info.width, info.height = 640, 480
    info.k = k.ravel().tolist()
    info.d = [0.] * 5
    info.distortion_model = 'plumb_bob'
    node.info = info
    node.colors.append(rgb)
    marker = Marker()
    marker.header.frame_id = node.base
    marker.id, marker.ns, marker.type = 7, 'placed_item_visuals', Marker.CUBE
    marker.pose.position.x, marker.pose.position.z = .01, .12
    marker.pose.orientation.w = 1.
    marker.scale.x, marker.scale.y, marker.scale.z = size
    node._markers('placed', MarkerArray(markers=[marker]))
    node._matrix = lambda dst, src, stamp: np.eye(4) if src == node.base else camera
    node._capture('placed:placed_item_visuals:7')
    return mask


def test_capture_fit_overlay_and_save(node, tmp_path):
    node.set_parameters([rclpy.parameter.Parameter('results_directory', value=str(tmp_path))])
    mask = capture_synthetic(node)
    assert node.state == 'CAPTURED'
    assert node.snapshot['meta']['prompt_points']
    meta = node.snapshot['meta']
    node.result = fit_top(mask, np.asarray(meta['k']),
        np.asarray(meta['d']), np.asarray(meta['base_from_camera']),
        meta['prior_top_base_m'], meta['size_m'], meta['prior_yaw'])
    node.mask = mask
    node._draw()
    node._save()
    folders = list(tmp_path.iterdir())
    assert len(folders) == 1
    payload = json.loads((folders[0]/'metadata.json').read_text())
    assert payload['diagnostic_only']
    assert payload['result']['image_center_error_px'] == pytest.approx([0, 0], abs=1)
    with np.load(folders[0]/'frames.npz') as frames:
        assert 'depth_m' not in frames
        assert frames['rgb'].shape == (480, 640, 3)
        assert frames['mask'].any()
    assert (folders[0]/'preview.png').stat().st_size > 100


def test_selection_change_cannot_segment_old_capture(node):
    capture_synthetic(node)
    node._command(String(data=json.dumps({'action': 'segment', 'target': 'another'})))
    assert node.state == 'ERROR'
    assert 'selection changed' in node.message
    assert node.future is None


def test_clear_discards_late_worker_result(node):
    capture_synthetic(node)
    future = Future()
    node.future, node.worker_generation = future, node.generation
    node._command(String(data='{"action":"clear"}'))
    future.set_result(({'top_center_base_m': [1, 2, 3]}, np.ones((2, 2))))
    node._tick()
    assert node.snapshot is None and node.result is None and node.mask is None
    assert node.future is None
    assert node.state == 'IDLE'


def test_deleted_scene_marker_cannot_be_captured(node):
    capture_synthetic(node)
    node._markers('placed', MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
    with pytest.raises(ValueError, match='unavailable'):
        node._capture('placed:placed_item_visuals:7')
    assert node.snapshot is None


def test_image_decode_handles_padding_endian_and_bgr():
    raw = np.array([[1000, 1500, 9999], [2000, 2500, 9999]], dtype='>u2')
    msg = Image(height=2, width=2, step=6, encoding='16UC1', is_bigendian=1, data=raw.tobytes())
    np.testing.assert_allclose(image_array(msg), [[1., 1.5], [2., 2.5]])
    bgr = Image(height=1, width=1, step=4, encoding='bgr8', data=bytes([10, 20, 30, 0]))
    assert image_array(bgr).tolist() == [[[30, 20, 10]]]
    bgr.step = 2
    with pytest.raises(ValueError, match='row stride'):
        image_array(bgr)


def test_sam_adapter_validates_mask_and_uses_frozen_prompts(node, tmp_path, monkeypatch):
    mask = capture_synthetic(node)
    checkpoint = tmp_path / 'fake-test-checkpoint.pt'
    checkpoint.write_bytes(b'interface mock only; not a real model')
    node.set_parameters([rclpy.parameter.Parameter('sam_checkpoint', value=str(checkpoint))])
    calls = {}

    class Predictor:
        def __init__(self, model):
            calls['model'] = model

        def set_image(self, rgb):
            calls['rgb'] = rgb.copy()

        def predict(self, **kwargs):
            calls['prompts'] = kwargs
            # An empty high-score mask must not defeat the valid lower-score one.
            return np.array([np.zeros_like(mask), mask]), np.array([.99, .8]), None

    monkeypatch.setitem(sys.modules, 'torch', NS(inference_mode=nullcontext))
    monkeypatch.setitem(sys.modules, 'sam2', NS())
    monkeypatch.setitem(sys.modules, 'sam2.build_sam', NS(build_sam2=lambda *a, **kw: 'model'))
    monkeypatch.setitem(sys.modules, 'sam2.sam2_image_predictor', NS(SAM2ImagePredictor=Predictor))
    result, result_mask = node._segment(node.snapshot)
    assert result['sam_score'] == .8
    assert calls['prompts']['multimask_output']
    assert calls['prompts']['point_coords'].shape == (5, 2)
    assert calls['rgb'].shape == (480, 640, 3)
    assert np.array_equal(result_mask, mask)


def test_frame_mismatch_cannot_retain_old_capture(node):
    capture_synthetic(node)
    node.info.width = 320
    with pytest.raises(ValueError, match='geometry/frame mismatch'):
        node._capture('placed:placed_item_visuals:7')
    assert node.snapshot is None and node.result is None


def test_no_depth_subscription(node):
    assert not any('depth' in sub.topic_name and 'pointcloud_detection' not in sub.topic_name
                   for sub in node.subscriptions)
    assert not hasattr(node, 'depths')


@pytest.mark.parametrize('camera', [transform([0, 0, .8], [0, 0, 0, 1]),
                                  transform([0, 0, .8], [0, math.sin(math.pi/4), 0, math.cos(math.pi/4)])])
def test_invalid_plane_intersections_rejected(camera):
    k, _, size, prior, _, mask = scene()
    with pytest.raises(ValueError, match='behind|parallel'):
        fit_top(mask, k, np.zeros(5), camera, prior, size, 0)


def test_oblique_camera_recovers_recorded_height_and_yaw():
    k, _, size, _, _, _ = scene()
    angle, yaw = .25, .3
    # Downward camera tilted about base Y, with optical axis toward the box.
    rotation_y = transform([0, 0, 0], [0, math.sin(angle/2), 0, math.cos(angle/2)])
    camera = transform([.6*math.sin(angle), 0, .2+.6*math.cos(angle)], [1, 0, 0, 0])
    camera[:3, :3] = rotation_y[:3, :3] @ camera[:3, :3]
    prior = top_points(size, transform([0, 0, .12], [0, 0, math.sin(yaw/2), math.cos(yaw/2)]))
    pix = project(prior, np.linalg.inv(camera), k, np.zeros(5))
    import cv2
    mask = np.zeros((480, 640), np.uint8)
    cv2.fillConvexPoly(mask, np.rint(pix[1:]).astype(np.int32), 1)
    fit = fit_top(mask, k, np.zeros(5), camera, prior, size, yaw)
    assert fit['top_center_base_m'] == pytest.approx([0, 0, .2], abs=.002)
    assert fit['yaw_rad'] == pytest.approx(yaw, abs=.02)
