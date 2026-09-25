import time
from types import SimpleNamespace as NS
import pytest
from safe_servo_visualization.pickup_supervisor_node import PickupSupervisor

@pytest.mark.parametrize('motion,robot,age,allow,blocked', [
    ('PREPARED',2,0,True,False), ('PREPARED',2,0,False,True),
    ('PREPARED',1,0,True,True), ('PREPARED',2,10,True,True),
    ('EXECUTING',2,0,True,True), ('PLANNING',2,0,True,True)])
def test_manual_release_only_allows_stopped_faulted_prepared_motion(motion,robot,age,allow,blocked):
    node=NS(state='FAULT', FAULT='FAULT', ACTIVE={'EXECUTING'},
            motion_status={'state':motion}, robot_state=robot,
            robot_state_time=time.monotonic()-age, status_timeout=1.,
            orchestrator_status={'cycle':{'state':'FAULT'}})
    assert bool(PickupSupervisor._manual_control_busy_reason(node,allow)) == blocked
    node.orchestrator_status['cycle']['state']='EXECUTING'
    assert PickupSupervisor._manual_control_busy_reason(node,allow)
