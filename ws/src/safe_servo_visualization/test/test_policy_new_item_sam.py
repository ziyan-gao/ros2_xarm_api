import time
from types import SimpleNamespace as NS
import pytest
from safe_servo_visualization.top_face_automation import TopFaceAutomation


@pytest.mark.parametrize('owner,state', [('policy', 'LOCALIZING'),
                                        ('policy', 'WAITING_NEXT_ITEM'),
                                        ('test', 'ESTIMATING')])
def test_new_item_owner_and_token_required(owner, state):
    node = NS(inspection={'owner': 'new_item', 'request_id': 'token'},
              motion_status={'estimate': {'state': 'SAM_REFINEMENT', 'sam_request_id': 'token'},
                             owner: {'state': state}},
              motion_seen={'estimate': time.monotonic(), owner: time.monotonic()})
    assert TopFaceAutomation._inspection_owners(node) == {'estimate', owner}
    node.motion_status['estimate']['sam_request_id'] = 'obsolete'
    with pytest.raises(ValueError):
        TopFaceAutomation._inspection_owners(node)
    node.motion_status['estimate']['sam_request_id'] = 'token'
    node.motion_seen[owner] = 0.
    with pytest.raises(ValueError):
        TopFaceAutomation._inspection_owners(node)
