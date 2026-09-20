import json

from safe_servo_visualization.policy_result_recorder import PolicyResultRecorder


def test_result_is_valid_after_each_event_and_only_commits_are_operations(tmp_path):
    recorder = PolicyResultRecorder(tmp_path, {'target_util': 0.7})
    summary = {'packed_item_count': 1, 'utilization_including_clearance': 0.25}
    for kind in ('planner_started', 'planner_finished', 'operation_completed', 'fault'):
        recorder.append(kind, {'operation': 'pack'}, summary)
        data = json.loads(recorder.path.read_text())
        assert data['summary'] == summary
    assert len(data['events']) == 4
    assert len(data['operations']) == 1
    assert data['operations'][0]['timestamp'].endswith('+00:00')
    assert not recorder.path.with_suffix('.json.tmp').exists()
    assert PolicyResultRecorder(tmp_path, {}).path != recorder.path
