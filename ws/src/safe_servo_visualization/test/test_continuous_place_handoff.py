from types import SimpleNamespace as NS

import pytest

from safe_servo_visualization.place_pipeline_node import PlacePipeline


class Future:
    def add_done_callback(self, callback):
        self.callback = callback

    def result(self):
        if isinstance(self.value, Exception):
            raise self.value
        return self.value

    def complete(self, value):
        self.value = value
        self.callback(self)


class Client:
    def __init__(self):
        self.calls = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        future = Future()
        self.calls.append(future)
        return future


def make_pipeline(monkeypatch):
    clock = [10.]
    monkeypatch.setattr(
        'safe_servo_visualization.place_pipeline_node.time.monotonic', lambda: clock[0])
    node = object.__new__(PlacePipeline)
    node.state = node.LOAD_PRE_PLACE
    node.operation_id = 3
    node.continuous_transport = True
    node.continuous_ack_started = None
    node.continuous_ack_received = False
    node.expected_motion_operation_id = 7
    node.expected_supervisor_operation_id = 8
    node.motion_status = {'state':'PREPARED','operation_id':7,'target':'transfer'}
    node.supervisor_status = {'state':'SUCCEEDED','operation_id':8,
        'operation_kind':'loading','direct_transfer_succeeded':True}
    node.loading_succeeded_at = None
    node.post_loading_settle = .75
    node.accept_direct_transfer = Client()
    node.start_place = Client()
    node.start_loading = Client()
    node.publish_status = lambda: None
    node.get_logger = lambda: NS(info=lambda *a: None,error=lambda *a: None)
    return node,clock


def test_waits_for_ack_and_matching_status_before_servo(monkeypatch):
    node,clock = make_pipeline(monkeypatch)
    node._tick_loading()
    node._tick_loading()
    assert len(node.accept_direct_transfer.calls) == 1
    assert not node.start_place.calls
    node.accept_direct_transfer.calls[0].complete(NS(success=True))
    node._tick_loading()  # Response alone does not override PREPARED telemetry.
    assert not node.start_place.calls
    node.motion_status['state'] = 'SUCCEEDED'
    node._tick_loading()  # Start the normal settling window only after handoff.
    assert not node.start_place.calls
    clock[0] += .8
    node._tick_loading()
    assert node.state == node.CONTACT_PLACE
    assert len(node.start_place.calls) == 1
    assert not node.start_loading.calls  # Descent must not be executed twice.


def test_status_before_ack_response_also_waits(monkeypatch):
    node,clock = make_pipeline(monkeypatch)
    node._tick_loading()
    node.motion_status['state'] = 'SUCCEEDED'
    node._tick_loading()
    assert node.loading_succeeded_at is None
    assert not node.start_place.calls
    node.accept_direct_transfer.calls[0].complete(NS(success=True))
    node._tick_loading()
    clock[0] += .8
    node._tick_loading()
    assert len(node.start_place.calls) == 1


@pytest.mark.parametrize('response', [None, NS(success=False,message='rejected'), RuntimeError('failed')])
def test_failed_ack_never_starts_servo(monkeypatch, response):
    node,_ = make_pipeline(monkeypatch)
    node._tick_loading()
    node.accept_direct_transfer.calls[0].complete(response)
    assert node.state == node.FAULT
    assert not node.start_place.calls


def test_ack_status_timeout_does_not_start_servo(monkeypatch):
    node,clock = make_pipeline(monkeypatch)
    node._tick_loading()
    node.accept_direct_transfer.calls[0].complete(NS(success=True))
    clock[0] += 5.1
    node._tick_loading()
    assert node.state == node.FAULT
    assert 'timed out' in node.fault
    assert not node.start_place.calls


def test_different_coordinator_operation_is_not_accepted(monkeypatch):
    node,_ = make_pipeline(monkeypatch)
    node._tick_loading()
    node.accept_direct_transfer.calls[0].complete(NS(success=True))
    node.motion_status.update(state='SUCCEEDED',operation_id=9)
    node._tick_loading()
    assert node.state == node.FAULT
    assert not node.start_place.calls


@pytest.mark.parametrize('change', ['abort','new_operation'])
def test_late_ack_cannot_resume_aborted_or_new_cycle(monkeypatch, change):
    node,_ = make_pipeline(monkeypatch)
    node._tick_loading()
    if change == 'abort':
        node.state = node.FAULT
    else:
        node.operation_id += 1
    node.accept_direct_transfer.calls[0].complete(NS(success=True))
    assert not node.continuous_ack_received
    assert not node.start_place.calls


def test_legacy_loading_needs_no_new_ack(monkeypatch):
    node,clock = make_pipeline(monkeypatch)
    node.continuous_transport = False
    node._tick_loading()
    clock[0] += .8
    node._tick_loading()
    assert len(node.start_place.calls) == 1
    assert not node.accept_direct_transfer.calls
