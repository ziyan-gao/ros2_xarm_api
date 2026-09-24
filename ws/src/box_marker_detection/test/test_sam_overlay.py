import numpy as np
from box_marker_detection.sam_overlay import projected_mask
from safe_servo_visualization.top_face_geometry import mask_plane_contours
from box_marker_detection.detector_node import BoxMarkerDetector
from collections import deque
from types import SimpleNamespace as NS
from unittest.mock import Mock


def test_display_waits_for_exact_tf_without_blocking_perception(monkeypatch):
    clock = [1.]
    monkeypatch.setattr('box_marker_detection.detector_node.time.monotonic', lambda: clock[0])
    from sensor_msgs.msg import Image
    msg = Image()
    msg.header.frame_id = 'camera'
    msg.header.stamp.sec = 101
    frame = np.zeros((2, 2, 3), np.uint8)
    node = NS(video_pending=deque([(msg, frame, np.eye(3), np.zeros(5), 1.)]),
              sam_projection=dict(color_stamp=100., base_frame='link_base'),
              tf_buffer=NS(can_transform=Mock(return_value=False)), render_video=Mock())
    BoxMarkerDetector.flush_video(node)
    assert len(node.video_pending) == 1
    node.render_video.assert_not_called()
    node.tf_buffer.can_transform.return_value = True
    BoxMarkerDetector.flush_video(node)
    assert not node.video_pending
    node.render_video.assert_called_once()


def test_display_timeout_does_not_freeze_video(monkeypatch):
    monkeypatch.setattr('box_marker_detection.detector_node.time.monotonic', lambda: 1.3)
    from sensor_msgs.msg import Image
    msg = Image()
    msg.header.stamp.sec = 101
    node = NS(video_pending=deque([(msg, None, None, None, 1.)]),
              sam_projection=dict(color_stamp=100., base_frame='link_base'),
              tf_buffer=NS(can_transform=Mock(return_value=False)), render_video=Mock())
    BoxMarkerDetector.flush_video(node)
    node.render_video.assert_called_once()
    assert not node.video_pending


def test_reprojection_moves_with_camera_and_expires_from_capture():
    payload = dict(color_stamp=100., polygons=[[[-.1, -.1, 1.], [.1, -.1, 1.],
                                               [.1, .1, 1.], [-.1, .1, 1.]]])
    k = np.array([[100., 0, 50.], [0, 100., 50.], [0, 0, 1.]])
    tf = np.eye(4)
    first = projected_mask(payload, 101., tf, k, np.zeros(5), (100, 100))
    assert first[50, 50] and not first[20, 20]
    tf[0, 3] = .2
    moved = projected_mask(payload, 109.9, tf, k, np.zeros(5), (100, 100))
    assert moved[50, 70] and not moved[50, 50]
    assert projected_mask(payload, 110., tf, k, np.zeros(5), (100, 100)) is None
    assert projected_mask(payload, 99., tf, k, np.zeros(5), (100, 100)) is None
    tf[2, 3] = -2.
    assert projected_mask(payload, 101., tf, k, np.zeros(5), (100, 100)) is None


def test_mask_plane_round_trip_preserves_hole():
    mask = np.zeros((100, 100), np.uint8)
    mask[20:80, 20:80] = 1
    mask[40:60, 40:60] = 0
    k = np.array([[100., 0, 50.], [0, 100., 50.], [0, 0, 1.]])
    polygons = mask_plane_contours(mask, k, np.zeros(5), np.eye(4), 1.)
    result = projected_mask(dict(color_stamp=10., polygons=polygons), 11.,
                            np.eye(4), k, np.zeros(5), mask.shape)
    assert result[30, 30] and not result[50, 50]
    assert np.count_nonzero(result != mask.astype(bool)) < 100
