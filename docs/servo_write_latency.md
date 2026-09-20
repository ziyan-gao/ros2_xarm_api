# Servo-J write latency containment

## Evidence and limits

The observed 217 ms and 422 ms stalls occurred inside `set_servo_angle_j`, after
successful Cartesian planning. The controller's 100 ms watchdog requested STOP
after the blocked call returned. The short-XY waypoint branch is unrelated to
this driver's implementation and has been reverted separately.

A passive TCP snapshot also showed one retransmitted 47-byte packet on the
robot's command connection, with an RTO of 204 ms. A Servo-J command here is
47 bytes (7-byte header plus ten floats). This is consistent with a command
retransmission, but cumulative counters do not establish the timing or cause of
the individual stalls. Do not assume a force-sensor or IK failure from this.

## Changes

`patches/xarm_sdk_bounded_servoj.patch` applies to the pinned C++ SDK submodule
inside the image, not the separately installed Python SDK:

- Command TCP sockets enable `TCP_NODELAY`; report sockets are unchanged. This
  avoids Nagle/delayed-ACK interactions, not packet loss or retransmissions.
- Servo-J mutex acquisition has a 5 ms budget. If it cannot obtain the lock,
  it fails without sending that target.
- Linux Servo-J sends use `MSG_DONTWAIT | MSG_NOSIGNAL`, with a 5 ms send budget.
  Partial writes continue from the unsent byte offset, never replay the frame.
  An incomplete partial frame closes that stream to prevent protocol corruption.
- The entire Servo-J operation has a 50 ms monotonic budget, including lock,
  send and reply. The reply uses the remaining budget, not the generic SDK
  multi-second timeout. A response arriving beyond the deadline is not success.
- Failed calls latch out subsequent Servo-J requests on that SDK instance.
  Neither late acknowledgements, mode changes nor reconnects clear this latch.
  The existing driver STOP request, controller cancellation and lifecycle fault
  latch remain in force. Ordinary non-Servo SDK commands are otherwise unchanged.
- Fault/slow-call logs split `lock_us`, `send_us`, `reply_us`, `total_us`, `phase`
  and `ret`. Healthy fast cycles do not print these diagnostics.

These are software waiting budgets, **not hard real-time guarantees**. Scheduler
stalls and logging can exceed them. This does not guarantee timely physical STOP
over a failed network: the existing STOP command uses that connection too, and a
command already handed to TCP cannot be recalled. Hardware emergency stopping
and a clear work area remain necessary. No worker queue, stale-target replay,
automatic fault clearing or automatic trajectory continuation is introduced.

This contains failures earlier; it does **not** guarantee uninterrupted operation
on a faulty link. If deadlines keep failing, inspect the logged phase and TCP
retransmissions, cabling/switch, host scheduling and controller response. Do not
increase the deadline merely to keep an experiment running.

## Deployment

Source changes alone, `colcon build` of `/workspace/ws`, or restarting the old
container do not rebuild `/opt/xarm_ws`'s SDK. With the robot safely stopped and
any held item supported, build the image from `src/ros2_xarm_api`:

```bash
docker compose build ros2_cv
```

The image build applies the SDK patch, rebuilds its consumers and runs the C++
mock/loopback tests. Deploy only after the build succeeds and the workcell is
ready; starting this service starts the robot stack:

```bash
docker compose up -d --no-deps --force-recreate ros2_cv
```

Before any real cycle, inspect hardware fault/state and controller ownership.
This implementation has offline verification only; do not treat image creation
as robot validation. Start with a supervised single low-speed cycle, not an
automatic repeating experiment. Preserve the first fault log if it stops.

## Offline tests

`tests/servo_write_deadline_test.cpp` covers healthy payload encoding, lock
contention, send failure, late send, late success, reply timeout, fault latching,
STOP availability after a latch, warning compatibility and remaining-budget
accounting. TCP cases use only a self-created ephemeral loopback server, including
a reply delayed by 220 ms and a non-reading peer that exhausts the send buffer.
The partial-send test checks that the corrupted stream is closed and cannot
accept another frame. No robot address or robot connection is used.
