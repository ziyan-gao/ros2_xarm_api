import time
from types import SimpleNamespace

from lifecycle_msgs.msg import State

from safe_servo_visualization.pickup_supervisor_node import PickupSupervisor


class FakeFuture:
    def __init__(self, result):
        self._result = result
        self.callback = None

    def result(self):
        return self._result

    def add_done_callback(self, callback):
        self.callback = callback


class FakeClient:
    def __init__(self, result=None):
        self.ready = True
        self.result = result
        self.requests = []
        self.futures = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        future = FakeFuture(self.result)
        self.futures.append(future)
        return future


class FakeLogger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass


def _supervisor(state):
    supervisor = object.__new__(PickupSupervisor)
    supervisor.state = state
    supervisor.fault = ''
    supervisor.trajectory_controller = 'uf850_traj_controller'
    supervisor.joint_state_broadcaster = 'joint_state_broadcaster'
    supervisor.hardware_component = (
        'uf_robot_hardware/UFRobotSystemHardware')
    supervisor.ros2_control_mode = 1
    supervisor.mode_transition_timeout = 5.0
    supervisor.mode_retry_interval = 0.25
    supervisor.mode_ready_samples = 3
    supervisor.status_timeout = 1.0
    supervisor.robot_state = None
    supervisor.robot_mode = None
    supervisor.robot_error = None
    supervisor.robot_state_time = None
    supervisor.mode_wait_target = None
    supervisor.mode_wait_label = ''
    supervisor.mode_wait_callback = None
    supervisor.mode_wait_deadline = None
    supervisor.mode_wait_last_command = None
    supervisor.mode_wait_command_pending = False
    supervisor.mode_wait_clear_pending = False
    supervisor.mode_wait_ready_count = 0
    supervisor.set_mode_client = FakeClient(SimpleNamespace(ret=0))
    supervisor.set_state_client = FakeClient(SimpleNamespace(ret=0))
    supervisor.clean_error_client = FakeClient(SimpleNamespace(ret=0))
    supervisor.get_logger = lambda: FakeLogger()
    supervisor._fault = lambda message: setattr(supervisor, 'fault', message)
    return supervisor


def _controller(name, state):
    return SimpleNamespace(name=name, state=state)


def _hardware_component(state_id, label):
    return SimpleNamespace(
        name='uf_robot_hardware/UFRobotSystemHardware',
        state=SimpleNamespace(id=state_id, label=label))


def test_direct_retreat_deactivates_controllers_before_hardware():
    supervisor = _supervisor(PickupSupervisor.PREPARING_RETREAT)
    supervisor.controller_switch_client = FakeClient()
    hardware_calls = []
    supervisor._deactivate_retreat_hardware = (
        lambda: hardware_calls.append(True))
    response = SimpleNamespace(controller=[
        _controller('uf850_traj_controller', 'active'),
        _controller('joint_state_broadcaster', 'active'),
    ])

    supervisor._retreat_controller_state_received(FakeFuture(response))

    assert hardware_calls == []
    request = supervisor.controller_switch_client.requests[-1]
    assert request.activate_controllers == []
    assert request.deactivate_controllers == [
        'uf850_traj_controller', 'joint_state_broadcaster']

    supervisor.controller_switch_client.futures[-1]._result = (
        SimpleNamespace(ok=True))
    supervisor.controller_switch_client.futures[-1].callback(
        supervisor.controller_switch_client.futures[-1])
    assert hardware_calls == [True]


def test_restore_configures_unconfigured_hardware_before_activation():
    supervisor = _supervisor(PickupSupervisor.RESTORING_CONTROL)
    transitions = []
    supervisor._set_restore_hardware_state = (
        lambda state_id, label, callback:
        transitions.append((state_id, label, callback.__name__)))
    response = SimpleNamespace(component=[
        _hardware_component(State.PRIMARY_STATE_UNCONFIGURED, 'unconfigured')])

    supervisor._restore_hardware_state_received(FakeFuture(response))

    assert transitions == [(
        State.PRIMARY_STATE_INACTIVE,
        'inactive',
        '_restore_hardware_configured')]


def _complete_mode_state_command(supervisor):
    mode_future = supervisor.set_mode_client.futures[-1]
    mode_future.callback(mode_future)
    state_future = supervisor.set_state_client.futures[-1]
    state_future.callback(state_future)


def _confirm_mode(supervisor, mode):
    supervisor.robot_state = 0
    supervisor.robot_mode = mode
    supervisor.robot_error = 0
    for _ in range(supervisor.mode_ready_samples):
        supervisor.robot_state_time = time.monotonic()
        supervisor._mode_readiness_tick()


def test_restore_confirms_robot_mode_before_activating_hardware():
    supervisor = _supervisor(PickupSupervisor.RESTORING_CONTROL)
    supervisor.hardware_state_client = FakeClient()
    controller_restore_calls = []
    supervisor._restore_trajectory_controller = (
        lambda: controller_restore_calls.append(True))
    response = SimpleNamespace(component=[
        _hardware_component(State.PRIMARY_STATE_INACTIVE, 'inactive')])

    supervisor._restore_hardware_state_received(FakeFuture(response))

    assert supervisor.hardware_state_client.requests == []
    assert supervisor.mode_wait_target == 1
    _complete_mode_state_command(supervisor)
    _confirm_mode(supervisor, 1)

    request = supervisor.hardware_state_client.requests[-1]
    assert request.name == supervisor.hardware_component
    assert request.target_state.id == State.PRIMARY_STATE_ACTIVE
    assert request.target_state.label == 'active'
    assert controller_restore_calls == []

    result = SimpleNamespace(
        ok=True,
        state=SimpleNamespace(
            id=State.PRIMARY_STATE_ACTIVE, label='active'))
    future = supervisor.hardware_state_client.futures[-1]
    future._result = result
    future.callback(future)
    assert controller_restore_calls == [True]


def test_restore_deactivates_unexpectedly_active_hardware_before_mode_change():
    supervisor = _supervisor(PickupSupervisor.RESTORING_CONTROL)
    supervisor.hardware_state_client = FakeClient()
    response = SimpleNamespace(component=[
        _hardware_component(State.PRIMARY_STATE_ACTIVE, 'active')])

    supervisor._restore_hardware_state_received(FakeFuture(response))

    request = supervisor.hardware_state_client.requests[-1]
    assert request.target_state.id == State.PRIMARY_STATE_INACTIVE
    assert supervisor.mode_wait_target is None

    result = SimpleNamespace(
        ok=True,
        state=SimpleNamespace(
            id=State.PRIMARY_STATE_INACTIVE, label='inactive'))
    future = supervisor.hardware_state_client.futures[-1]
    future._result = result
    future.callback(future)
    assert supervisor.mode_wait_target == 1


def test_mode_wait_periodically_reissues_mode_until_telemetry_matches():
    supervisor = _supervisor(PickupSupervisor.PREPARING_RETREAT)
    ready_calls = []

    supervisor._begin_mode_wait(0, 'direct retreat',
                                lambda: ready_calls.append(True))
    assert len(supervisor.set_mode_client.requests) == 1
    _complete_mode_state_command(supervisor)

    supervisor.robot_state = 0
    supervisor.robot_mode = 1
    supervisor.robot_error = 0
    supervisor.robot_state_time = time.monotonic()
    supervisor.mode_wait_last_command = time.monotonic() - 1.0
    supervisor._mode_readiness_tick()
    assert len(supervisor.set_mode_client.requests) == 2
    assert ready_calls == []

    _complete_mode_state_command(supervisor)
    _confirm_mode(supervisor, 0)
    assert ready_calls == [True]


def test_mode_wait_clears_c52_before_confirming_direct_motion():
    supervisor = _supervisor(PickupSupervisor.PREPARING_RETREAT)
    supervisor.robot_state = 0
    supervisor.robot_mode = 0
    supervisor.robot_error = 52
    supervisor.robot_state_time = time.monotonic()
    ready_calls = []

    supervisor._begin_mode_wait(0, 'direct retreat',
                                lambda: ready_calls.append(True))
    assert len(supervisor.clean_error_client.requests) == 1
    assert len(supervisor.set_mode_client.requests) == 0

    clear_future = supervisor.clean_error_client.futures[-1]
    clear_future.callback(clear_future)
    supervisor.robot_error = 0
    _confirm_mode(supervisor, 0)
    assert ready_calls == [True]
