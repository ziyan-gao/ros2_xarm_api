from types import SimpleNamespace

from safe_servo_visualization.random_stable_loading_node import (
    RandomStableLoadingNode,
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
