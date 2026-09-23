# Real-Platform Policy Loading: Experimental Setup and Procedure

This document describes the implemented experimental procedure and the settings
associated with the recorded session
`policy_20260920T075203_632473Z_c7c451a0.json` on September 20, 2026.
Settings can differ between sessions; use each result file's configuration and
runtime events when comparing experiments.

## 1. Experimental setup

The experiment is an **online robotic packing task with human-supplied incoming
items**. A six-axis UFACTORY UF850 robot, equipped with a vacuum gripper,
wrist-mounted RGB-D camera, and force–torque sensing, transfers cardboard boxes
onto a pallet. A separate staging area provides temporary buffer slots for
rearrangement.

The human operator presents **one new item at a time** in the designated camera
observation/pickup area. The robot measures that item and determines how to
accommodate it using the current packing state. The system is not given the
dimensions or arrival order of future items.

The robot's saved **observation pose** is an end-effector configuration used to
view the pickup area; it is distinct from the position where the operator places
the incoming item.

| Parameter | Setting |
|---|---|
| Packing volume | 450 × 550 × 450 mm |
| XY planning resolution | 5 mm |
| Z planning resolution | 5 mm |
| XY clearance | 25 mm, one-sided virtual dimension inflation |
| Height tolerance for support estimation | 0 mm |
| COM-bound ratio | 0.3 |
| Maximum placement candidates | 80 |
| Policy inference device | CPU |
| MCTS iterations | 100 |
| Maximum unpack operations searched | 6 |
| MCTS maximum children parameter | 3 |
| A* time limit | 30 s |
| MCTS utilization target | 0.75 |
| Buffer-slot camera inspection | Disabled |

The checkpoint is
`train_outputs/cardboard_xy_random_clearance20/policy_step.pth` in the Neuromeka
repository. Its filename does not override the runtime clearance: the experiment
uses the YAML value of 25 mm.

The YAML defaults to rearrangement disabled, but the selected session's planning
events record `rearrangement_enabled: true`, reflecting the panel setting used
during the experiment. The MCTS utilization target is a search criterion, not a
claim that the whole experiment automatically terminates at that utilization.

## 2. Object pose and dimension estimation

Each incoming item undergoes visual estimation followed by contact-based height
correction.

### 2.1 Visual estimation

The robot first reaches its saved observation pose. Aligned depth images are
converted into points in the robot-base frame using camera calibration and the
camera-to-robot transformation. A region-of-interest crop isolates the pickup
area.

The depth-only estimator fits a near-horizontal top plane using RANSAC. A
minimum-area rectangle fitted to the top-plane points provides:

- Object XY center.
- In-plane orientation, or yaw.
- Footprint dimensions `dx` and `dy`.

The pipeline collects **20 distinct timestamped estimates**, rather than
repeatedly reading one cached result. The estimates are averaged after passing
the configured stability checks: position spread within 5 mm, orientation spread
within 2°, and dimension spread within 10 mm. Invalid or stale observations do
not count as acceptable samples. Detection also has geometric and crop limits;
absence of a catalog whitelist does not mean that every visible object is
accepted by perception.

### 2.2 Contact-based height correction

Using the visual estimate, the robot approaches a pre-pick pose above the item
and descends under force monitoring, with the vacuum initially off. After
contact is confirmed and descent stops, the TCP height is read:

```text
dz_measured = z_TCP_contact - configured_contact_offset
```

The offset corresponds to the calibrated empty-table contact reference. The
estimated object height and vertical pose are updated accordingly; the visual
XY position and yaw are retained. The height correction uses the stopped TCP
position, rather than another multi-frame depth-height estimate.

Object information is latched at the contact pose. If a valid placement is
available, pickup can proceed by activating the vacuum without repeating the
observation and probing sequence.

### 2.3 Dimensions supplied to the policy

ROS rounds XY downward and Z upward to 5 mm increments. With dimensions in mm:

```text
dx_policy = 5 × floor(dx_measured / 5)
dy_policy = 5 × floor(dy_measured / 5)
dz_policy = 5 × ceil(dz_measured / 5)
```

For example, an XY dimension of 149 mm becomes 145 mm, while a height of 149 mm
becomes 150 mm.

The policy does **not** require an exact match to a cardboard-size dictionary.
Unrounded measured dimensions are retained separately for utilization reporting.
The environment's existing space-pruning and feasibility rules remain enabled.

## 3. Packing decisions and rearrangement

The packing environment maintains the placed-item geometry, height map, and
support information. It generates empty-space placement candidates and evaluates
their feasibility, including containment, clearance, and stability.

The learned policy receives normalized item dimensions, candidate-space
geometry, and a feasibility mask. It selects a placement and one of two
footprint orientations: the original XY orientation or a 90° rotation. The
height map is used by the environment to construct candidates and feasibility
masks; it should not be described as a direct policy-network input here.

Clearance is represented using virtual dimensions. With the selected session's
25 mm clearance and 5 mm-aligned planning dimensions:

```text
dx_virtual = dx_policy + 25 mm
dy_virtual = dy_policy + 25 mm
dz_virtual = dz_policy
```

This is planning-space inflation, not a guarantee that every physical gap will
measure exactly 25 mm. Estimation error, dimension rounding, grasp error, and
placement error can affect the observed gap.

If direct placement has no feasible action and rearrangement is enabled,
**MCTS searches for a rearrangement plan**, and **A* optimizes its execution
sequence**. The resulting operations may include:

- **Pack:** move the incoming item or a buffered item onto the pallet.
- **Unpack:** remove a pallet item and place it in a staging slot.
- **Repack:** move an existing pallet item to a different pallet pose.

Slot inspection is disabled in the selected configuration. Buffered items are
retrieved using their saved placement/grasp information, without a new
camera-based pose estimate. Items in the slots should therefore not be assumed
to have been re-localized after an unobserved displacement.

## 4. Robot motion generation

Packing decisions and robot motion planning are separate. The policy specifies
an item placement in the pallet frame; ROS transforms that placement into TCP
targets and generates executable trajectories.

### 4.1 Nominal loaded transfer

The nominal loaded motion follows:

```text
Pickup → vertical lift → elevated transfer → pre-place pose
       → guarded final descent → release
```

The elevated transfer height accounts for the item dimensions and grasp
transform. The nominal item-bottom clearance is the container height plus
20 mm: **470 mm above the pallet reference**, not simply a fixed TCP height.
Rounded transitions and orientation-dependent item geometry can require
additional TCP height.

The lift, transfer, and non-servo descent are connected using sampled Cartesian
paths with rounded transitions where feasible. Required yaw changes are
performed in the elevated portion of the route. MoveIt's Cartesian-path service
solves the corresponding joint trajectory; timing is then checked and adjusted
before execution through the joint trajectory controller.

This use of MoveIt for Cartesian path computation is distinct from unrestricted
free-space motion planning. The normal transport is waypoint-driven.

### 4.2 Pallet–buffer crossings

For pallet–buffer crossings, additional observation-side waypoints guide the
path around the workspace. If the nominal route is infeasible, bounded waypoint
alternatives are evaluated. Compatible, previously executed overhead path
sections may also be reused, with new endpoint connections and renewed
validation. Reuse is conditional, not guaranteed for every crossing.

After placement, the robot performs a slow clearance retreat before proceeding
to an elevated handoff pose or returning toward observation, depending on the
operation sequence. A cross-area approach to the next item uses the corresponding
waypoint route rather than assuming that a direct low-level lateral motion is
appropriate.

**Continuous motion is the nominal approach, but fallback trajectories may
contain intermediate stops.** Planning success alone is not execution success;
the controller result and final-state checks are part of the operation workflow.

## 5. Human-in-the-loop experimental procedure

1. Initialize an empty virtual packing state consistent with the physical pallet
   and staging slots.
2. The operator presents a new item in the designated pickup area, withdrawing
   from the robot workspace before the next robot operation.
3. The robot obtains its visual pose and contact-corrected height.
4. The policy chooses direct packing or, when necessary and enabled, an MCTS+A*
   rearrangement sequence.
5. ROS executes the required pack, unpack, and repack operations.
6. The packing state is updated after physical-operation confirmation.
7. The operator supplies the next item, and the process repeats.

Thus, the human supplies items while the system determines their placements and
rearrangements. Future incoming items are not included as a known sequence in
the packing decision. The session ends when the operator ends the trial or the
system cannot proceed; perception, planning, or execution faults may interrupt
the automatic cycle and require operator intervention.

The random robustness-test panel is a separate testing mode and should not be
described as the policy-loading experimental protocol.

## 6. Recorded data and evaluation metrics

Each session is saved to a timestamped JSON file under:

```text
src/ros2_xarm_api/results/policy_loading/
```

The records include operation events, completed operations, planner invocation
times and durations, faults, packed-item counts, and utilization snapshots.
Runtime events should be checked alongside the initial configuration, because
panel settings such as rearrangement enablement can change during a session.

Three utilization measures are distinguished. Let `P` be the items currently on
the pallet and `V_container = 450 × 550 × 450 mm³`:

```text
U_measured = Σ[i in P] (dx_measured,i × dy_measured,i × dz_measured,i) / V_container
U_planning = Σ[i in P] (dx_policy,i   × dy_policy,i   × dz_policy,i)   / V_container
U_virtual  = Σ[i in P] (dx_virtual,i  × dy_virtual,i  × dz_virtual,i)  / V_container
```

| JSON metric | Meaning |
|---|---|
| `utilization_measured` | Unrounded measured volume, excluding clearance |
| `utilization_planning` | Rounded planning volume, excluding clearance |
| `utilization_including_clearance` | Clearance-inflated virtual volume |

Only items currently on the pallet contribute; buffered items do not. The
recorded packed-item count describes pallet occupancy, not the cumulative
number of pack actions, which can include repeated handling of the same item.

For reporting physical packing performance, use `utilization_measured`, while
noting that it is measurement-based rather than independently measured ground
truth. If measured-dimension metadata is absent in older runs, do not silently
treat rounded or virtual utilization as measured utilization.

The current MCTS target uses rounded dimensions **without clearance**, not the
measured-utilization metric. Legacy top-level utilization labels in result
files should not supersede the explicit metric names and `utilization_bases`.

## 7. Implementation references

- [Policy configuration](../../../neuromeka_bin_packing/configs/real_platform_policy.yaml)
- [Policy loader](../../../neuromeka_bin_packing/packing/real_platform_policy_loading.py)
- [Depth estimator](../ws/src/box_marker_detection/box_marker_detection/depth_refinement_node.py)
- [Pickup estimation pipeline](../ws/src/safe_servo_visualization/safe_servo_visualization/pickup_pipeline_node.py)
- [Pickup supervisor and contact correction](../ws/src/safe_servo_visualization/safe_servo_visualization/pickup_supervisor_node.py)
- [Transport geometry](../ws/src/safe_servo_visualization/safe_servo_visualization/transport_path.py)
- [Continuous transport executor](../ws/src/safe_servo_visualization/safe_servo_visualization/continuous_transport.py)
- [Policy geometry configuration notes](policy_geometry_config.md)
- [Measured utilization notes](policy_measured_utilization.md)
- [Cross-area route search and reuse](cross_frame_search.md)
