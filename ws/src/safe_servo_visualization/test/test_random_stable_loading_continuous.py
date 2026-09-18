from types import SimpleNamespace

import pytest

from safe_servo_visualization.random_stable_loading_node import (
    RandomStableLoadingNode,
    round_down_to_increment,
    round_up_to_increment,
)


class FakeLoader:
    def __init__(self):
        self.pending = SimpleNamespace(item_id=9, sequence_id=4)

    def commit(self, sequence_id):
        assert sequence_id == self.pending.sequence_id
        self.pending = None
        return SimpleNamespace(FLB=SimpleNamespace(x=10, y=20, z=30))


class FakeLogger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass


class FakePlanningWorker:
    def __init__(self):
        self.calls = []

    def submit(self, function, **kwargs):
        self.calls.append((function, kwargs))
        return SimpleNamespace(done=lambda: False)


class FakeVisualizationBuilder:
    def __init__(self):
        self.calls = []

    def build(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        return 'frame'


class FakeVisualizationServer:
    def __init__(self):
        self.frames = []

    def push(self, frame):
        self.frames.append(frame)


@pytest.mark.parametrize(
    ('measured', 'expected'),
    [(140.0, 140.0), (141.0, 145.0), (149.0, 150.0), (150.0, 150.0)],
)
def test_packing_height_is_rounded_up_to_five_millimetres(measured, expected):
    assert round_up_to_increment(measured, 5) == expected


@pytest.mark.parametrize(
    ('measured', 'expected'),
    [(150.0, 150.0), (149.0, 145.0), (136.0, 135.0), (131.0, 130.0)],
)
def test_item_xy_is_rounded_down_to_five_millimetres(measured, expected):
    assert round_down_to_increment(measured, 5) == expected


def test_corrected_object_result_plans_only_during_requested_estimation():
    node = object.__new__(RandomStableLoadingNode)
    node.state = 'LOCALIZING'
    node.loader = SimpleNamespace(pending=None, plan=lambda **_kwargs: None)
    node.packing_height_resolution_mm = 5
    node.planning_worker = FakePlanningWorker()
    node.planning_future = None
    node.localization_started = 1.0
    node.fault = 'old'
    node.last_result = 'old'
    node.target_acknowledged = True
    node.get_logger = lambda: FakeLogger()
    node.publish_status = lambda: None
    message = SimpleNamespace(
        data=[4.0, 0.1, 0.2, 0.075, 0.0, 0.0, 0.0, 1.0,
              0.141, 0.149, 0.147])

    node.item_result_callback(message)

    assert node.state == 'PLANNING'
    assert len(node.planning_worker.calls) == 1
    assert node.planning_worker.calls[0][1] == {
        'item_id': 4,
        'dimensions_mm': (140.0, 145.0, 150.0),
    }


def test_unsolicited_corrected_object_result_is_ignored_while_idle():
    node = object.__new__(RandomStableLoadingNode)
    node.state = 'IDLE'
    node.loader = SimpleNamespace(pending=None)
    node.planning_worker = FakePlanningWorker()
    node.get_logger = lambda: FakeLogger()
    message = SimpleNamespace(
        data=[4.0, 0.1, 0.2, 0.075, 0.0, 0.0, 0.0, 1.0,
              0.141, 0.149, 0.147])

    node.item_result_callback(message)

    assert node.state == 'IDLE'
    assert node.planning_worker.calls == []


def test_success_at_observation_starts_next_stability_measurement():
    node = object.__new__(RandomStableLoadingNode)
    node.loader = FakeLoader()
    node.continuous_loading_enabled = True
    node.continuous_run_active = True
    node.localization_started = 1.0
    node.cycle_auto_start = True
    node.abort_requested = False
    node.abort_started = None
    node.start_request_pending = False
    node.expected_pipeline_operation_id = 8
    node.target_acknowledged = True
    node.get_logger = lambda: FakeLogger()
    node._push_visualization = lambda _title: None
    starts = []
    node._start_continuous_localization = lambda: starts.append(True)
    node.publish_status = lambda: None

    node._commit_succeeded_pick_place()

    assert starts == [True]
    assert node.loader.pending is None
    assert node.last_result == 'committed item 9 at (10, 20, 30) mm'


def test_waiting_for_next_item_has_no_localization_timeout():
    node = object.__new__(RandomStableLoadingNode)
    node.planning_future = None
    node.state = 'WAITING_NEXT_ITEM'
    node.localization_started = None
    node._set_fault = lambda reason: (_ for _ in ()).throw(
        AssertionError(reason))

    node.tick()

    assert node.state == 'WAITING_NEXT_ITEM'


def test_disabling_continuous_mode_stops_idle_item_wait():
    node = object.__new__(RandomStableLoadingNode)
    node.state = 'WAITING_NEXT_ITEM'
    node.loader = SimpleNamespace(pending=None)
    node.continuous_run_active = True
    node.cycle_auto_start = True
    node.localization_started = None
    clears = []
    node._clear_item_localization = lambda: clears.append(True)
    node.publish_status = lambda: None
    response = SimpleNamespace(success=False, message='')

    node.set_continuous_callback(
        SimpleNamespace(data=False), response)

    assert response.success is True
    assert node.state == 'IDLE'
    assert node.continuous_run_active is False
    assert clears == [True]


def test_status_reports_shared_random_loading_geometry():
    node = object.__new__(RandomStableLoadingNode)
    node.loader = SimpleNamespace(
        container_size=(400, 500, 300),
        clearance_mm=12,
        clearance_mode='one_sided',
        com_bound_ratio=0.25,
        height_tolerance=10.0,
        vertical_loading_filter_enabled=False,
    )
    node.transfer_corner_height = 0.33
    node.random_loading_config_path = '/config/random.yaml'
    payload = {}

    node._extend_status(payload, pending=None)

    assert payload['container_size_mm'] == [400, 500, 300]
    assert payload['clearance_mm'] == 12
    assert payload['clearance_mode'] == 'one_sided'
    assert payload['height_tolerance_mm'] == pytest.approx(10.0)
    assert payload['vertical_loading_filter_enabled'] is False
    assert payload['selection_pipeline'] == 'stable_then_random_at_minimum_z'
    assert payload['transfer_corner_height_m'] == pytest.approx(0.33)
    assert payload['random_loading_config_path'] == '/config/random.yaml'


def test_live_visualization_draws_virtual_item_dimensions():
    node = object.__new__(RandomStableLoadingNode)
    pending_item = object()
    node.loader = SimpleNamespace(
        env=object(),
        pending_item_for_visualization=lambda: pending_item,
    )
    node.visualization_builder = FakeVisualizationBuilder()
    node.visualization_server = FakeVisualizationServer()
    node.visualization_fault = ''
    node.get_logger = lambda: FakeLogger()

    node._push_visualization('virtual footprint')

    assert node.visualization_server.frames == ['frame']
    args, kwargs = node.visualization_builder.calls[0]
    assert args == (node.loader.env, 'virtual footprint')
    assert kwargs['highlighted_items'] == [pending_item]
    assert kwargs['virtual_boxes'] is True
