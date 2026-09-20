"""Pure geometry checks for X-first overhead transport, without robot access."""
import numpy as np
import pytest

from safe_servo_visualization.transport_path import rounded_translation_waypoints
from safe_servo_visualization.transport_alternatives import blend_translation_segments, buffer_transfer_waypoints


Q = np.array([1., 0., 0., 0.])


def test_buffer_waypoints_replace_old_observation_y_with_destination_y():
    a, b = buffer_transfer_waypoints([.2, .5, .15], [.6, -.4, .2], [.45, .02, .1], .65)
    assert np.allclose(a, [.30, .5, .65])
    assert np.allclose(b, [.30, -.4, .65])
    c, d = buffer_transfer_waypoints([.6, -.4, .2], [.2, .5, .15], [.45, .02, .1], .65)
    assert np.allclose(c, b) and np.allclose(d, a)


@pytest.mark.parametrize('offset', [-.1, 0., .1])
def test_candidate_offset_moves_both_waypoints_from_new_150mm_rail(offset):
    a, b = buffer_transfer_waypoints([.2, .5, .15], [.6, -.4, .2],
                                    [.45, .02, .1], .65, offset)
    assert a[0] == pytest.approx(.30 + offset)
    assert b[0] == pytest.approx(.30 + offset)


def test_x_first_route_rounds_elbow_but_preserves_vertical_columns_and_clearance():
    start, end = np.array([.2, .5, .15]), np.array([.3, -.6, .2])
    destinations = [[.2, .5, .65], [.4, .5, .65], [.4, .4, .65],
                    [.3, -.6, .65], end]
    samples = rounded_translation_waypoints(start, destinations, Q, .61)
    assert np.allclose(samples[-1][0], end)
    assert all(np.allclose(q, Q) for p, q in samples)
    for p, _ in samples:
        if p[2] < .61-1e-9:
            assert min(np.linalg.norm(p[:2]-start[:2]), np.linalg.norm(p[:2]-end[:2])) < 1e-9
        assert p[2] <= .65+1e-9
    # Before reaching the observation side, route first traverses X at source Y.
    assert any(p[0] > .3 and abs(p[1]-.5) < 1e-9 for p, q in samples)
    # The elbow is rounded, not an exact 90-degree turn requiring a full stop.
    assert not any(np.allclose(p, destinations[1], atol=1e-6) for p, q in samples)
    assert max(np.linalg.norm(b[0]-a[0]) for a, b in zip(samples, samples[1:])) < .01


def test_vertical_corner_radius_is_capped_by_clearance_margin():
    samples = rounded_translation_waypoints([0, 0, .1], [[0, 0, .62], [.4, 0, .62]], Q, .61)
    assert all(p[2] >= .61-1e-9 for p, q in samples if p[0] > 1e-9)


def test_duplicate_or_collinear_waypoints_are_finite_and_reach_exact_end():
    samples = rounded_translation_waypoints([0, 0, .7],
        [[0, 0, .7], [.1, 0, .7], [.1, 0, .7], [.2, 0, .7]], Q, .6)
    assert all(np.isfinite(p).all() for p, q in samples)
    assert np.allclose(samples[-1][0], [.2, 0, .7])
    assert rounded_translation_waypoints([0, 0, .7], [[0, 0, .7]], Q, .6) == []


def test_reversal_is_not_shortcut_by_corner_rounding():
    samples = rounded_translation_waypoints([0, 0, .7], [[.2, 0, .7], [0, 0, .7]], Q, .6)
    assert any(np.allclose(p, [.2, 0, .7]) for p, q in samples)
    assert np.allclose(samples[-1][0], [0, 0, .7])


@pytest.mark.parametrize('kwargs', [{'radius': 0}, {'step': -1}, {'safe_z': float('nan')}])
def test_invalid_blend_parameters_rejected(kwargs):
    options = dict(safe_z=.6, radius=.04, step=.005)
    options.update(kwargs)
    with pytest.raises(ValueError):
        rounded_translation_waypoints([0, 0, .1], [[0, 0, .7]], Q, **options)


def test_grouping_does_not_merge_rotation_or_moveit_with_translation():
    qb = np.array([0., 1., 0., 0.])
    segments = [('cartesian', [0, 0, .7], Q), ('cartesian', [.2, 0, .7], Q),
                ('cartesian', [.2, 0, .7], qb), ('cartesian', [.2, .2, .7], qb),
                ('cartesian', [.2, .2, .1], qb), ('moveit', [.3, .4, .7], qb)]
    grouped = blend_translation_segments(segments, Q)
    assert [s[0] for s in grouped] == ['cartesian_blend', 'cartesian', 'cartesian_blend', 'moveit']
    assert len(grouped[0][1]) == len(grouped[2][1]) == 2
    assert np.allclose(grouped[1][2], qb)


def test_quaternion_sign_does_not_create_unnecessary_segment_boundary():
    result = blend_translation_segments([
        ('cartesian', [0, 0, .7], Q), ('cartesian', [.2, 0, .7], -Q)], Q)
    assert len(result) == 1 and result[0][0] == 'cartesian_blend'
