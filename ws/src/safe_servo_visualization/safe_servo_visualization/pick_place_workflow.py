"""Shared manipulation routing. Poses/records stay owned by source adapters.

This module never commands hardware. The returned recipe composes the existing
guarded pickup, carry, placement, and retreat executors; no blind release step.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class PickAndPlace:
    source: str
    destination: str
    return_to_observation: bool = True
    pre_pick_pose: object = None

    def __post_init__(self):
        if self.source not in ('incoming', 'slot', 'pallet', 'carried'):
            raise ValueError('unsupported PickAndPlace source')
        if self.destination not in ('pallet', 'slot'):
            raise ValueError('unsupported PickAndPlace destination')
        if not isinstance(self.return_to_observation, bool):
            raise TypeError('return_to_observation must be a boolean')
        # Known sources resolve a recorded contact pose + dimensions in their
        # adapter. Never silently substitute incoming-item estimation.
        if self.source in ('slot', 'pallet') and self.pre_pick_pose is None:
            raise ValueError('known sources require a pre-pick source record')

    @property
    def pickup_service(self):
        return {
            'incoming': '/pickup_pipeline/start_for_transport',
            'slot': '/staging_slots/retrieve_chained',
            'pallet': '/pickup_supervisor/start_for_transport',
            'carried': None,
        }[self.source]

    @property
    def placement_service(self):
        if self.destination == 'slot':
            return '/staging_slots/store' if self.return_to_observation else '/staging_slots/store_chained'
        return ('/place_pipeline/start_continuous' if self.return_to_observation
                else '/place_pipeline/start_continuous_chained')

    @property
    def stages(self):
        acquire = (() if self.source == 'carried' else
                   ('estimate_at_observation', 'contact_pick') if self.source == 'incoming'
                   else ('overhead_pre_pick', 'contact_pick'))
        return acquire + ('carry_transfer', 'contact_place', 'slow_clearance',
                          'overhead_observation' if self.return_to_observation else 'overhead_handoff')


def pick_and_place(source, destination, *, pre_pick_pose=None, return_to_observation=True):
    """Build an immutable recipe; executors confirm each phase before advancing."""
    return PickAndPlace(source, destination, return_to_observation, pre_pick_pose)
