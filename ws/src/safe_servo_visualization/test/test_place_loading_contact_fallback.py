from safe_servo_visualization.place_pipeline_node import PlacePipeline


def test_loading_contact_fallback_skips_safe_servo_place():
    pipeline = object.__new__(PlacePipeline)
    pipeline.supervisor_status = {
        'operation_kind': 'loading',
        'operation_id': 12,
        'state': 'SUCCEEDED',
        'place_fallback_used': True,
        'place_fallback_reason': 'contact during linear loading',
    }
    pipeline.expected_supervisor_operation_id = 12
    pipeline.loading_succeeded_at = 1.0
    observations = []
    pipeline._begin_observation_motion = lambda: observations.append(True)

    class Logger:
        def warning(self, _message):
            pass

    pipeline.get_logger = lambda: Logger()

    pipeline._tick_loading()

    assert observations == [True]
    assert pipeline.loading_succeeded_at is None
