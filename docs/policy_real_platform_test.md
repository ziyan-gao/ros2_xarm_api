# Learned policy on the real platform (without MCTS/A*)

The first real-platform stage uses the cardboard policy checkpoint with the
450 x 550 x 450 mm pallet model and 20 mm virtual X/Y item clearance. MCTS and
A* are explicitly disabled in
`neuromeka_bin_packing/configs/real_platform_policy.yaml`.

The policy coordinator uses the existing contact-corrected object-information
pipeline. It rounds measured Z upward to 5 mm, sends the measured item as the
only policy input, and publishes the selected pallet-frame corner using the
existing `/random_stable_loading/target` message format. The policy pallet is
updated only after PickAndPlace reports success.

## Build and start

```bash
cd ~/tase2026_revision/src/ros2_xarm_api
docker compose build ros2_cv
docker compose up
```

The policy status and Three.js view are available at:

```bash
ros2 topic echo /policy_loading/status
# Browser: http://127.0.0.1:8766
```

Do not activate random loading and policy loading at the same time. They share
the downstream pallet-target topic by design.

## Plan without grasping or placing

This still runs object-information estimation, so the robot goes to the
observation/pre-pick region and touches the top face. It does not close the
gripper or execute PickAndPlace.

```bash
ros2 service call /policy_loading/plan std_srvs/srv/Trigger '{}'
```

After status becomes `TARGET_READY`, inspect:

```bash
ros2 topic echo /random_stable_loading/target
ros2 topic echo /policy_loading/status
```

To execute that pending target:

```bash
ros2 service call /policy_loading/start_pick_place std_srvs/srv/Trigger '{}'
```

## Run one complete policy cycle

```bash
ros2 service call /policy_loading/start std_srvs/srv/Trigger '{}'
```

This estimates object information, evaluates the learned policy, publishes and
applies the target, then starts PickAndPlace. On robot failure the predicted
placement is not committed to the policy pallet.

Reset only while neither coordinator nor PickAndPlace is active:

```bash
ros2 service call /policy_loading/reset_pallet std_srvs/srv/Trigger '{}'
```
