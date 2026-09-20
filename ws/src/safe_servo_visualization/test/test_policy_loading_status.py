import json
from types import SimpleNamespace

from geometry_msgs.msg import Pose
import numpy as np
from packing_env.data_type.geometry import Orthogonal3D, Point3D
from packing_env.data_type.item import Item
import pytest

from safe_servo_visualization.policy_loading_node import PolicyLoadingNode


@pytest.mark.parametrize('step_index', [0, 1, 2])
def test_every_policy_unpack_uses_raised_handoff_store(step_index):
    operation = SimpleNamespace(kind='unpack', source='pallet',
                                source_item=object(), step_index=step_index,
                                step_count=3)
    workflow = PolicyLoadingNode._operation_workflow(operation)
    assert not workflow.return_to_observation
    assert workflow.placement_service == '/staging_slots/store_chained'
    returning_client = object()
    calls = []
    node = SimpleNamespace(
        workflow=workflow, staging_status={'slots': []}, holding_slots={},
        staging_store_selection_pub=SimpleNamespace(publish=lambda msg: None),
        staging_store_return_client=object(),
        staging_store_client=returning_client,
        _defer_service=lambda client, label: calls.append(client))
    PolicyLoadingNode._start_staging_store(node, operation)
    assert calls == [returning_client]
    assert node.state == 'REARRANGE_STORE'


@pytest.mark.parametrize('kind,source', [('pack', 'incoming'),
                                         ('pack', 'holding'), ('repack', 'pallet')])
@pytest.mark.parametrize('step_index', [0, 2])
def test_other_policy_operations_keep_existing_return_behavior(kind, source, step_index):
    operation = SimpleNamespace(kind=kind, source=source, source_item=object(),
                                step_index=step_index, step_count=3)
    workflow = PolicyLoadingNode._operation_workflow(operation)
    assert workflow.return_to_observation is (step_index == 2)


def test_policy_container_size_comes_from_yaml_config():
    assert PolicyLoadingNode._configured_container_size(
        {'container_size': [300, 350, 350]},
        fallback=(450, 550, 450),
    ) == (300, 350, 350)


def test_policy_transfer_height_uses_own_container_and_margin():
    assert PolicyLoadingNode._configured_transfer_height({}, (450, 550, 450)) == .47
    assert PolicyLoadingNode._configured_transfer_height(
        {'transfer_clearance_mm': 30}, (450, 550, 450)) == .48
    for margin in (0, -1, 19, float('nan'), float('inf')):
        with pytest.raises(ValueError):
            PolicyLoadingNode._configured_transfer_height(
                {'transfer_clearance_mm': margin}, (450, 550, 450))


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
        container_size=(np.int64(450), np.int64(500), np.int64(570)),
        device='cpu',
        clearance_mm=np.int64(20),
        xy_resolution_mm=np.int64(5),
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
    node.pallet_records = {node._item_key(item): {
        'corner_mm': [100, 200, 50], 'size_mm': [200, 100, 150],
        'release_tcp_rpy_rad': [np.pi, 0., .7]}}

    message = node._pallet_item_retrieval_message(operation)

    assert message.data[:7] == pytest.approx(
        [12.0, 1.2, 2.25, 3.2, np.pi, 0.0, .7])
    assert message.data[7:11] == pytest.approx([0.2, 0.1, 0.15, 0.03])
    assert message.data[11:] == pytest.approx([0.0, 3.47, 0.0])


@pytest.mark.parametrize('scene_key', ['placed_item_ids', 'placed_item_visual_ids'])
def test_empty_policy_inventory_blocks_scene_items_without_erasing_them(scene_key):
    node = object.__new__(PolicyLoadingNode)
    node.loader = SimpleNamespace(env=SimpleNamespace(container=SimpleNamespace(placed_items=[])))
    node.scene_status = {scene_key: ['placed_item_1']}
    assert 'reconcile' in node._empty_inventory_conflict()
    assert node.scene_status[scene_key] == ['placed_item_1']
    node.loader.env.container.placed_items = [object()]
    assert node._empty_inventory_conflict() == ''
    node.loader.env.container.placed_items = []
    node.simulation_enabled = True
    assert node._empty_inventory_conflict() == ''


def test_policy_records_rotated_physical_dimensions_and_release_orientation():
    node = object.__new__(PolicyLoadingNode)
    node.pallet_records = {}
    node.supervisor_status = {'pallet_release_rpy_rad': [3.14, .01, 1.2]}
    node._set_fault = lambda reason: pytest.fail(reason)
    # Physical dimensions differ from planner padded dimensions.
    values = [1., 9., 10., 20., 30., 1., 140., 100., 150., 180., 140., 150.]
    assert node._save_pallet_record((9,), values)
    record = node.pallet_records[(9,)]
    assert record['size_mm'] == [100., 140., 150.]
    assert record['corner_mm'] == [10., 20., 30.]
    assert record['release_tcp_rpy_rad'] == [3.14, .01, 1.2]


def test_policy_missing_release_orientation_does_not_invent_pickup_yaw():
    node = object.__new__(PolicyLoadingNode)
    node.pallet_records = {}
    node.supervisor_status = {}
    faults = []
    node._set_fault = faults.append
    assert not node._save_pallet_record((9,), [])
    assert not node.pallet_records
    assert 'release TCP orientation' in faults[0]


@pytest.mark.parametrize('kind,source', [('unpack', 'pallet'), ('repack', 'pallet'), ('pack', 'holding')])
def test_policy_commit_preserves_physical_inventory_mapping(kind, source):
    node = object.__new__(PolicyLoadingNode)
    old = SimpleNamespace(to_key=lambda: (1,))
    new = SimpleNamespace(to_key=lambda: (2,))
    node.active_operation = SimpleNamespace(
        kind=kind, source=source, source_item=old,
        target_box=None if kind == 'unpack' else new, sequence_id=7,
        item_id=9, has_target=kind != 'unpack',
        target_values=[7., 9., 10., 20., 30., 0., 140., 100., 150., 180., 140., 150.],
        step_index=0, step_count=1)
    node.pallet_records = {(1,): {'old': True}}
    node.holding_slots = {(1,): 3} if source == 'holding' else {}
    node.placed_obstacle_ids = {(1,): 'placed_item_1'}
    node.active_slot = 3
    node.supervisor_status = {'pallet_release_rpy_rad': [3.14, 0., .5]}
    commits = []
    node.loader = SimpleNamespace(commit_current_operation=lambda seq: commits.append(seq) or True)
    node._update_pending_obstacle_mapping = lambda: None
    node._push_visualization = lambda _: None
    node._finish_rearrangement_plan = lambda: None
    node._complete_rearrangement_operation()
    assert commits == [7]
    if kind == 'unpack':
        assert node.holding_slots == {(1,): 3}
        assert not node.pallet_records
    else:
        assert node.pallet_records[(2,)]['release_tcp_rpy_rad'] == [3.14, 0., .5]
        assert not node.holding_slots
        assert node.pending_obstacle_key == (2,)
        if kind == 'repack':
            assert (1,) not in node.pallet_records


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


def test_scene_placed_item_ids_include_visual_only_markers():
    node = object.__new__(PolicyLoadingNode)
    node.scene_status = {
        'placed_item_ids': ['placed_item_2'],
        'placed_item_visual_ids': ['placed_item_2', 'placed_item_5'],
    }

    assert node._scene_placed_item_ids() == {
        'placed_item_2', 'placed_item_5'}


def test_pending_policy_item_maps_to_new_visual_only_marker():
    node = object.__new__(PolicyLoadingNode)
    item_key = (1, 20, 30, 40, 100, 120, 80)
    node.pending_obstacle_key = item_key
    node.placed_ids_before_operation = {'placed_item_2'}
    node.scene_status = {
        'placed_item_ids': [],
        'placed_item_visual_ids': ['placed_item_2', 'placed_item_7'],
    }
    node.placed_obstacle_ids = {}

    node._update_pending_obstacle_mapping()

    assert node.placed_obstacle_ids[item_key] == 'placed_item_7'
    assert node.pending_obstacle_key is None
