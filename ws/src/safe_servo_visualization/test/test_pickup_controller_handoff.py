import time
from types import SimpleNamespace

from lifecycle_msgs.msg import State

from safe_servo_visualization.pickup_supervisor_node import PickupSupervisor
from std_srvs.srv import Trigger


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

    def error(self, _message):
        pass


class FakeTimer:
    def __init__(self, callback):
        self.callback = callback
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


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
    supervisor.post_restore_settle = 0.75
    supervisor.post_restore_ready_samples = 5
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
    supervisor.restore_wait_sequence = None
    supervisor.restore_wait_deadline = None
    supervisor.restore_settle_started = None
    supervisor.restore_settle_ready_count = 0
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


def test_post_restore_requires_stable_robot_and_joint_telemetry_before_finish():
    supervisor = _supervisor(PickupSupervisor.RESTORING_CONTROL)
    supervisor.joint_state_ready_samples = 3
    supervisor.joint_state_ready_timeout = 5.0
    supervisor.joint_state_sequence = 3
    supervisor.last_joint_state_time = time.monotonic()
    supervisor.robot_state_time = time.monotonic()
    supervisor.robot_state = 0
    supervisor.robot_mode = 1
    supervisor.robot_error = 0
    completed = []
    supervisor._finish_retreat = lambda: completed.append(True)
    supervisor._begin_post_restore_joint_state_wait()
    supervisor.restore_wait_sequence = 0

    supervisor._restore_readiness_tick()

    assert completed == []
    assert supervisor.restore_settle_started is not None

    supervisor.restore_settle_started -= 0.8
    for _ in range(supervisor.post_restore_ready_samples):
        supervisor.last_joint_state_time = time.monotonic()
        supervisor.robot_state_time = time.monotonic()
        supervisor._restore_readiness_tick()

    assert completed == [True]


def test_post_restore_instability_resets_consecutive_ready_samples():
    supervisor = _supervisor(PickupSupervisor.RESTORING_CONTROL)
    supervisor.restore_settle_started = time.monotonic() - 1.0
    supervisor.restore_wait_deadline = time.monotonic() + 5.0
    supervisor.restore_settle_ready_count = 4
    supervisor.last_joint_state_time = time.monotonic()
    supervisor.robot_state_time = time.monotonic()
    supervisor.robot_state = 0
    supervisor.robot_mode = 0
    supervisor.robot_error = 0
    completed = []
    supervisor._finish_retreat = lambda: completed.append(True)

    supervisor._restore_readiness_tick()

    assert supervisor.restore_settle_ready_count == 0
    assert completed == []


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


def test_c52_during_place_descent_stops_servo_and_preserves_item():
    supervisor = _supervisor(PickupSupervisor.DESCENDING)
    supervisor.operation_kind = 'place'
    supervisor.planning_scene_status = {
        'attached_item_id': 'carried_item_7'}
    supervisor.direct_target_z = 0.47
    supervisor.robot_tcp_xyz = (0.3, -0.2, 0.25)
    supervisor.enable_client = FakeClient(
        SimpleNamespace(success=True, message='disabled'))
    supervisor.ft_enable_client = FakeClient(SimpleNamespace(ret=0))
    supervisor.clean_warn_client = FakeClient(SimpleNamespace(ret=0))
    supervisor.publish_status = lambda: None
    retreats = []
    supervisor._begin_direct_retreat = lambda: retreats.append(True)
    supervisor.ft_recovery_required = False
    supervisor.ft_recovery_reason = ''
    supervisor.c52_retreat_active = False
    supervisor.c52_clear_attempts = 0
    supervisor.c52_tcp_snapshot = None
    supervisor.direct_tcp_z_offset = 0.024

    supervisor._begin_c52_interruption()

    assert supervisor.state == PickupSupervisor.C52_STOPPING
    assert supervisor.ft_recovery_required is True
    assert supervisor.c52_retreat_active is True
    assert supervisor.c52_tcp_snapshot == (0.3, -0.2, 0.25)
    request = supervisor.enable_client.requests[-1]
    assert request.data is False
    future = supervisor.enable_client.futures[-1]
    future.callback(future)
    assert supervisor.ft_enable_client.requests[-1].data == 0
    disable_ft = supervisor.ft_enable_client.futures[-1]
    disable_ft.callback(disable_ft)
    clear_error = supervisor.clean_error_client.futures[-1]
    clear_error.callback(clear_error)
    clear_warn = supervisor.clean_warn_client.futures[-1]
    clear_warn.callback(clear_warn)
    assert retreats == [True]
    assert supervisor.c52_clear_attempts == 1


def test_c52_loaded_retreat_stops_if_ft_disable_is_rejected():
    supervisor = _supervisor(PickupSupervisor.C52_STOPPING)
    supervisor.ft_recovery_reason = 'xArm C52'
    supervisor.ft_enable_client = FakeClient(SimpleNamespace(ret=1))
    supervisor.clean_warn_client = FakeClient(SimpleNamespace(ret=0))
    supervisor._begin_direct_retreat = lambda: None

    supervisor._disable_ft_for_c52_retreat()
    disable_ft = supervisor.ft_enable_client.futures[-1]
    disable_ft.callback(disable_ft)

    assert 'disable FT sensor was rejected' in supervisor.fault
    assert supervisor.clean_error_client.requests == []


def test_c52_retreat_completion_latches_fault_without_releasing_item():
    supervisor = _supervisor(PickupSupervisor.RESTORING_CONTROL)
    supervisor.post_retreat_fault = ''
    supervisor.c52_retreat_active = True
    supervisor.c52_tcp_snapshot = (0.3, -0.2, 0.25)
    supervisor.ft_recovery_reason = 'xArm C52'
    supervisor.publish_status = lambda: None

    supervisor._finish_retreat()

    assert supervisor.state == PickupSupervisor.FAULT
    assert supervisor.c52_retreat_active is False
    assert 'item was preserved at the recovery waypoint' in supervisor.fault


def test_unconfigured_hardware_is_already_released_for_c52_retreat():
    supervisor = _supervisor(PickupSupervisor.PREPARING_RETREAT)
    released = []
    supervisor._retreat_hardware_is_released = lambda: released.append(True)
    response = SimpleNamespace(component=[
        _hardware_component(State.PRIMARY_STATE_UNCONFIGURED, 'unconfigured')])

    supervisor._retreat_hardware_state_received(FakeFuture(response))

    assert released == [True]


def test_ft_recovery_rejects_zeroing_while_item_is_attached():
    supervisor = _supervisor(PickupSupervisor.FAULT)
    supervisor.planning_scene_status = {
        'attached_item_id': 'carried_item_7'}

    response = supervisor.recover_ft_sensor_callback(
        None, Trigger.Response())

    assert response.success is False
    assert 'cannot zero FT sensor' in response.message


def test_unloaded_ft_recovery_verifies_force_then_restores_ros_control():
    supervisor = _supervisor(PickupSupervisor.FAULT)
    supervisor.planning_scene_status = {'attached_item_id': ''}
    supervisor.ft_enable_client = FakeClient(SimpleNamespace(ret=0))
    supervisor.ft_zero_client = FakeClient(SimpleNamespace(ret=0))
    supervisor.clean_warn_client = FakeClient(SimpleNamespace(ret=0))
    supervisor.enable_client = FakeClient()
    supervisor.ft_recovery_settle = 0.5
    supervisor.ft_recovery_max_attempts = 2
    supervisor.ft_recovery_required = True
    supervisor.ft_recovery_reason = 'C52'
    supervisor.ft_recovery_timer = None
    supervisor.post_retreat_fault = ''
    supervisor.c52_retreat_active = False
    supervisor.c52_clear_attempts = 0
    supervisor.c52_tcp_snapshot = None
    supervisor.last_force_time = None
    supervisor.force_timeout = 0.5
    supervisor.publish_status = lambda: None
    timers = []

    def create_timer(_period, callback):
        timer = FakeTimer(callback)
        timers.append(timer)
        return timer

    supervisor.create_timer = create_timer
    restores = []
    supervisor._restore_ros2_control_mode = lambda: restores.append(True)

    response = supervisor.recover_ft_sensor_callback(
        None, Trigger.Response())
    assert response.success is True

    disable = supervisor.ft_enable_client.futures[-1]
    disable.callback(disable)
    clear_error = supervisor.clean_error_client.futures[-1]
    clear_error.callback(clear_error)
    clear_warn = supervisor.clean_warn_client.futures[-1]
    clear_warn.callback(clear_warn)
    enable = supervisor.ft_enable_client.futures[-1]
    enable.callback(enable)
    timers[-1].callback()
    zero = supervisor.ft_zero_client.futures[-1]
    zero.callback(zero)
    supervisor.robot_state_time = time.monotonic()
    supervisor.robot_error = 0
    supervisor.last_force_time = time.monotonic()
    timers[-1].callback()

    assert restores == [True]
    assert supervisor.ft_recovery_restore_active is True

    supervisor.state = PickupSupervisor.RESTORING_CONTROL
    supervisor._finish_retreat()
    assert supervisor.state == PickupSupervisor.IDLE
    assert supervisor.ft_recovery_required is False
    assert supervisor.fault == ''
