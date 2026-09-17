import json
from types import SimpleNamespace

from geometry_msgs.msg import Pose
import numpy as np
from packing_env.data_type.geometry import Orthogonal3D, Point3D
from packing_env.data_type.item import Item
import pytest

from safe_servo_visualization.policy_loading_node import PolicyLoadingNode


def test_policy_container_size_comes_from_yaml_config():
    assert PolicyLoadingNode._configured_container_size(
        {'container_size': [300, 350, 350]},
        fallback=(450, 550, 450),
    ) == (300, 350, 350)


@pytest.mark.parametrize(
    'value', ([300, 350], [300, 350, 0], 'invalid'))
def test_policy_container_size_rejects_invalid_config(value):
    with pytest.raises(ValueError, match='container_size'):
        PolicyLoadingNode._configured_container_size(
            {'container_size': value},
            fallback=(450, 550, 450),
        )


def test_policy_status_converts_numpy_scalars_to_json_types():
    node = object.__new__(PolicyLoadingNode)
    node.loader = SimpleNamespace(
        checkpoint_path='/tmp/policy.pt',
        device='cpu',
        clearance_mm=np.int64(20),
        agent=SimpleNamespace(device='cpu'),
    )
    pending = SimpleNamespace(
        box=SimpleNamespace(
            FLB=SimpleNamespace(
                x=np.int64(10), y=np.int64(20), z=np.int64(30)),
            rot=np.bool_(True),
            Virtual_Dim=SimpleNamespace(
                raw=lambda: (
                    np.int64(200), np.int64(190), np.int64(110))),
        ),
        raw_dim=SimpleNamespace(
            raw=lambda: (
                np.int64(176), np.int64(162), np.int64(110))),
        action_index=np.int64(4),
        ems_index=np.int64(2),
        predicted_value=np.float32(0.447744),
    )
    payload = {}

    node._extend_status(payload, pending)

    encoded = json.dumps(payload)
    decoded = json.loads(encoded)
    assert decoded['target_corner_mm'] == [10, 20, 30]
    assert decoded['rotate_item_90_deg'] is True
    assert decoded['policy_action_index'] == 4
    assert decoded['policy_ems_index'] == 2
    assert decoded['policy_predicted_value'] == float(np.float32(0.447744))


def test_pallet_retrieval_target_uses_item_top_center_in_pallet_frame():
    class TransformBuffer:
        def lookup_transform(self, *_args):
            return SimpleNamespace(transform=SimpleNamespace(
                translation=SimpleNamespace(x=1.0, y=2.0, z=3.0),
                rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ))

    node = object.__new__(PolicyLoadingNode)
    node.tf_buffer = TransformBuffer()
    node.pallet_unpack_approach_height = 0.47
    item = Item(
        FLB=Point3D(100, 200, 50),
        Dim=Orthogonal3D(200, 100, 150),
    )
    operation = SimpleNamespace(sequence_id=12, source_item=item)

    message = node._pallet_item_retrieval_message(operation)

    assert message.data[:7] == pytest.approx(
        [12.0, 1.2, 2.25, 3.2, np.pi, 0.0, 0.0])
    assert message.data[7:11] == pytest.approx([0.2, 0.1, 0.15, 0.03])
    assert message.data[11:] == pytest.approx([0.0, 3.47])


def test_simulation_route_uses_fixed_pick_and_place_descents():
    node = object.__new__(PolicyLoadingNode)
    node.simulation_fixed_descent = 0.030
    node.pallet_unpack_approach_height = 0.47
    node.tf_buffer = SimpleNamespace(
        lookup_transform=lambda *_args: SimpleNamespace(
            transform=SimpleNamespace(
                translation=SimpleNamespace(z=0.0))))
    source = Pose()
    source.position.x = 0.2
    source.position.y = 0.3
    source.position.z = 0.15
    target = Pose()
    target.position.x = -0.1
    target.position.y = 0.4
    target.position.z = 0.25

    route = node._simulation_route(source, target)

    assert len(route) == 8
    assert route[1].position.z - route[2].position.z == pytest.approx(0.030)
    assert route[5].position.z - route[6].position.z == pytest.approx(0.030)
    assert route[0].position.z == pytest.approx(0.47)
    assert route[3].position.z == pytest.approx(0.47)
    assert route[4].position.z == pytest.approx(0.47)
    assert route[7].position.z == pytest.approx(0.47)


def test_simulation_pick_place_never_calls_physical_pipeline():
    node = object.__new__(PolicyLoadingNode)
    node.simulation_enabled = True
    node.fault = ''
    node.pick_place_client = SimpleNamespace(
        service_is_ready=lambda: (_ for _ in ()).throw(
            AssertionError('physical PickAndPlace client was accessed')))
    node._simulation_poses_for_pending = lambda: [Pose()]
    captured = {}
    node._start_simulation_motion = lambda poses, **kwargs: (
        captured.update(poses=poses, kwargs=kwargs) or True)
    response = SimpleNamespace(success=False, message='')

    result = node._start_pick_place(response)

    assert result.success
    assert captured['kwargs'] == {'completion': 'direct', 'label': 'pack'}


def test_simulation_mode_cannot_change_during_motion():
    node = object.__new__(PolicyLoadingNode)
    node.state = 'SIMULATING'
    node.simulation_enabled = True
    response = SimpleNamespace(success=False, message='')

    result = node.set_simulation_callback(
        SimpleNamespace(data=False), response)

    assert not result.success
    assert 'cannot change simulation mode' in result.message


def test_automatic_simulation_samples_cardboard_dimensions():
    sampled = Orthogonal3D(220, 170, 80)
    sampler = SimpleNamespace(sample=lambda count: [sampled])
    loader = SimpleNamespace(
        pending=None,
        rearrangement=None,
        env=SimpleNamespace(
            buffer=SimpleNamespace(data_sampler=sampler)),
    )

    class Worker:
        def submit(self, function, **kwargs):
            self.function = function
            self.kwargs = kwargs
            return 'planning-future'

    node = object.__new__(PolicyLoadingNode)
    node.loader = loader
    node.simulation_next_item_id = 1
    node.simulation_sample_count = 0
    node.simulation_incoming_x = 0.35
    node.simulation_incoming_y = 0.0
    node.simulation_support_z = 0.0
    node.planning_worker = Worker()
    node._plan_item = lambda **_kwargs: None
    node.publish_status = lambda: None
    node.get_logger = lambda: SimpleNamespace(info=lambda _message: None)

    item_id, dimensions = node._sample_cardboard_simulation_item(
        auto_start=True)

    assert item_id == 1
    assert dimensions == (220, 170, 80)
    assert node.state == 'PLANNING'
    assert node.cycle_auto_start
    assert node.simulation_sample_count == 1
    assert node.simulation_incoming['center'] == pytest.approx(
        (0.35, 0.0, 0.04))
    assert node.simulation_incoming['size'] == pytest.approx(
        (0.22, 0.17, 0.08))
    assert node.planning_worker.kwargs == {
        'item_id': 1,
        'dimensions_mm': (220, 170, 80),
    }


def test_simulation_start_samples_without_object_estimation_service():
    node = object.__new__(PolicyLoadingNode)
    node.simulation_enabled = True
    node.state = 'IDLE'
    node.loader = SimpleNamespace(pending=None)
    node.continuous_loading_enabled = True
    node._sample_cardboard_simulation_item = lambda **_kwargs: (
        4, (150, 100, 200))
    node.object_info_start_client = SimpleNamespace(
        service_is_ready=lambda: (_ for _ in ()).throw(
            AssertionError('object-estimation service was accessed')))
    response = SimpleNamespace(success=False, message='')

    result = node.start_loading_callback(None, response)

    assert result.success
    assert node.continuous_run_active
    assert 'cardboard item 4' in result.message
