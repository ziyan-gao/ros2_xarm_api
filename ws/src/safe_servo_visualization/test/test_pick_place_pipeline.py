import time

from safe_servo_visualization.pick_place_pipeline_node import PickPlacePipeline


class _UnexpectedPlaceClient:
    def service_is_ready(self):
        raise AssertionError('place service must not be inspected in pick-only mode')


def test_pick_only_finishes_after_pickup_and_does_not_start_place():
    pipeline = object.__new__(PickPlacePipeline)
    pipeline.state = 'PICKING'
    pipeline.started = time.monotonic()
    pipeline.timeout = 300.0
    pipeline.pick_only = True
    pipeline.expected_pickup_id = 4
    pipeline.expected_place_id = None
    pipeline.pickup_status = {'operation_id': 4, 'state': 'SUCCEEDED'}
    pipeline.place_status = {}
    pipeline.start_place = _UnexpectedPlaceClient()

    pipeline.tick()

    assert pipeline.state == 'SUCCEEDED'
