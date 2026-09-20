# Cross-frame waypoint fallback

For a fixed destination orientation, search in this order:

1. Observation-side rail, positive half-turn direction.
2. Absolute base-frame X = -350 mm rail, positive half-turn direction.
3. Observation-side rail, negative half-turn direction.
4. Absolute base-frame X = -350 mm rail, negative half-turn direction.

Each grid has a five-second monotonic deadline. XY offsets are -100 through
+100 mm, spaced by 25 mm. Timers advance stalled planning requests; late
responses from abandoned requests are ignored. Complete trajectory validation
is separate from the grid budget and retains its existing timeout. No search
transition interrupts an executing trajectory.

The two half-turn directions use intermediate orientations, not merely opposite
quaternion signs. Other endpoint orientation changes retain shortest-path
interpolation. Final targets and grasp transforms are not arbitrarily rotated.
Slot-store and inspection symmetry fallbacks can change the destination by 180
degrees using their existing grasp/slot bookkeeping, then repeat the grids.

Inspection preflight sends the successful rail, grid index, and turn direction
with its endpoint. The coordinator forwards this hint to the supervisor, which
replans that candidate from fresh measured joints and revalidates the complete
path. A preflight success is not permission to execute a stale trajectory.
Rejected wrist-unwinding candidates continue searching without moving.

Inspection yaw-180 preflight no longer requires the wrist endpoint to move
toward zero. It still rejects non-finite angles and intermediate absolute-angle
excursions greater than the larger start/end magnitude plus 0.05 rad. Rejection
logs include the start, end, peak and allowed peak in radians. The executor's
joint-limit, collision and timed-path validation remains unchanged. Same-area
grid fallback behavior is not changed by this wrist-policy adjustment.

Inspection endpoint height remains observation TCP Z minus 30 mm; XY backoff
remains 100 mm. Overhead crossings retain container clearance, followed by a
locally validated descent. Collision, joint-limit, and timed-path checks remain.

## Temporarily disabling slot inspection

`neuromeka_bin_packing/configs/real_platform_policy.yaml` now contains
`slot_inspection_enabled: false`. The unified launcher passes the same
`POLICY_LOADING_CONFIG_PATH` to the policy loader and staging coordinator.
The staging coordinator reads this flag at startup; when present, it takes
precedence over the legacy `STAGING_SLOT_INSPECTION_ENABLED` launch value.
Without the YAML key, the legacy value remains the fallback. Malformed flags
or unreadable configured files fail startup rather than silently changing mode.

With inspection disabled, slot retrieval skips the inspection move and depth
pose re-estimation and uses the saved slot pose and grasp. Contact pickup,
transfer and execution checks remain. This is a shared staging setting, so it
also affects test-panel slot retrieval, not just policy loading. New-item
object-information estimation is unchanged. Do not move a stored item and
assume the saved pose has been updated while inspection is off.

Set the flag back to `true` and restart the stack to restore inspection.
YAML edits are not hot-reloaded and do not release an existing fault or resume
a running operation. For standalone staging launches, provide
`slot_inspection_policy_config_path` explicitly to use the policy YAML.

The inspection target wire format extends the existing 14 fields with three
fields: rail (0/1), candidate index (-1 means nominal), direction (+1/-1).
The separate 21-field recorded-grasp format is unchanged. Restart the coordinator,
supervisor, and staging node together after deploying this change.

## Reusing an executed overhead path

The pickup supervisor now keeps up to 12 successfully executed and final-pose-
verified overhead joint-path sections in memory. This is controlled by its ROS
parameter `transport_path_reuse_enabled` (default `true`). Restarting the
supervisor clears this cache; the first crossing still needs normal planning.

For cross-area known-item approaches and carried pallet/buffer transfers, it
tries a compatible section in either direction before the normal waypoint
search. It never shifts the cached path in Cartesian space. Compatibility
includes the robot model limits, joint order, planning group/TCP, empty versus
loaded tool, exact carried-item dimensions and grasp transform, endpoint
orientation, current overhead clearance and ceiling. Item IDs alone do not
prevent reuse. Fixed-orientation operations cannot inherit a yaw excursion.
Grasp quaternions are normalized and sign-canonicalized, including half-turns,
so `q` and `-q` do not cause a cache miss. Dimensions and grasp translations
still require an exact match; actual grasp changes are not ignored.

FK recording belongs to a specific operation, route generation, and JTC
trajectory object. Only an accepted JTC goal followed by successful execution
and final-pose verification can populate the cache. SDK joint-line validation
clears this recording and does not add to it; clearing SDK mode flags during
restoration cannot make an earlier JTC path eligible for caching.

Inspection may change the pickup endpoint. The new endpoint is not overwritten:
the supervisor plans a fresh lift/approach connection at each end of the cached
section. The incoming connector is solved backwards from the cached boundary
joints, then reversed; a measured-start branch mismatch greater than 0.001 rad
rejects reuse. Only that small numerical start residual may be replaced by the
measured joints, and the resulting spline is subsequently checked. Cached and
connector joint seams must agree within 1e-6 rad. Conservative rest boundaries
are retained at the joins; reuse does not promise nonstop blending there.

Both connectors share a five-second planning budget. A timeout, incomplete
connector, incompatible branch, timing rejection, or failed collision/clearance
validation discards the cache candidate and resumes the original waypoint
search. Late callbacks are invalidated. There is no execution-time automatic
retry, fault clearing, or release added by this feature.

The entire assembled path is retimed and collision/FK-checked against the
current scene and carried item, including the cached section. Reuse avoids IK
planning for that section, not validation. Existing live-state and controller
handoff guards remain in force. Cache misses are normal after a changed grasp,
yaw, clearance, or restart.

Scope: this cache is in the shared **supervisor executor**. Inspection approaches
can reuse empty-tool sections there, but the staging node's separate inspection
preflight still runs its existing route search and must succeed first. No
cross-process preflight cache or persistent trajectory storage is introduced.

Look for `cross-frame path cache`, `cross-frame cache rejected`, and
`cross-frame cache accepted` in supervisor logs to distinguish hits, fallbacks,
and actual validated reuse. Offline tests do not certify a hardware run.
