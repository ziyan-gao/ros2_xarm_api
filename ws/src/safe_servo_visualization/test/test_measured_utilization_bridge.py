from types import SimpleNamespace as NS

import pytest
from builtin_interfaces.msg import Time

from safe_servo_visualization.pickup_pipeline_node import PickupPipeline
from safe_servo_visualization.policy_loading_node import PolicyLoadingNode
from test_random_stable_loading_continuous import FakeLogger, FakePlanningWorker


def test_publisher_keeps_existing_shape_and_adds_unrounded_measurements():
    node = object.__new__(PickupPipeline)
    node.finalized_result_published = False
    node.xy_dimension_rounding_mm = 5.
    raw = dict(box_id=0, x_m=.1, y_m=.2, center_z_m=.0756, top_z_m=.1512,
               yaw_rad=0., size_x_m=.1493, size_y_m=.1367, size_z_m=.1512)
    node.pickup_status = dict(corrected_object=raw)
    results, markers = [], []
    node.object_info_pub = NS(publish=results.append)
    node.object_info_marker_pub = NS(publish=markers.append)
    node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: Time()))
    node._set_fault = lambda reason: pytest.fail(reason)
    node._publish_finalized_object_info()
    assert list(results[0].data[8:11]) == pytest.approx([.145, .135, .1512])
    assert list(results[0].data[11:14]) == pytest.approx([.1493, .1367, .1512])
    assert markers[0].markers[0].scale.x == pytest.approx(.145)
    assert raw['size_x_m'] == .1493
    node._publish_finalized_object_info()
    assert len(results) == 1


@pytest.mark.parametrize('legacy', [False, True])
def test_policy_receives_separate_planning_and_measured_dimensions(legacy):
    node = object.__new__(PolicyLoadingNode)
    node.state = 'LOCALIZING'
    node.loader = NS(pending=None)
    node.packing_height_resolution_mm = 5
    node.planning_worker = FakePlanningWorker()
    node.get_logger = lambda: FakeLogger()
    node.publish_status = lambda: None
    values = [0., .1, .2, .0756, 0., 0., 0., 1., .145, .135, .1512]
    if not legacy:
        values += [.1493, .1367, .1512]
    node.item_result_callback(NS(data=values))
    _, kwargs = node.planning_worker.calls[0]
    assert kwargs['dimensions_mm'] == (145., 135., 155.)
    if legacy:
        assert 'measured_dimensions_mm' not in kwargs
    else:
        assert kwargs['measured_dimensions_mm'] == pytest.approx((149.3, 136.7, 151.2))


@pytest.mark.parametrize('rearrange', [False, True])
def test_both_policy_entry_points_forward_measurement(rearrange):
    node = object.__new__(PolicyLoadingNode)
    node.rearrangement_enabled = rearrange
    calls, records = [], []
    node.loader = NS(plan=lambda **kw: calls.append(('direct', kw)),
                     plan_with_rearrangement=lambda **kw: calls.append(('rearrange', kw)))
    node._record_result = lambda *args: records.append(args)
    node._plan_item(item_id=0, dimensions_mm=(145, 135, 155),
                    measured_dimensions_mm=(149.3, 136.7, 151.2))
    assert calls[0][0] == ('rearrange' if rearrange else 'direct')
    assert calls[0][1]['measured_dimensions_mm'] == (149.3, 136.7, 151.2)
    assert records[0][1]['measured_dimensions_mm'] == [149.3, 136.7, 151.2]
