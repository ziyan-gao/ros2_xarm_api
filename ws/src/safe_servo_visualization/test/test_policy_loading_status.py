import json
from types import SimpleNamespace

import numpy as np

from safe_servo_visualization.policy_loading_node import PolicyLoadingNode


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
