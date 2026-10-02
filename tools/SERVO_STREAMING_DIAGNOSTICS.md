# Servo streaming-write diagnostics

Raw Feetech-bus probes that bypass ROS entirely (same `Bus` class as
`calibrate_arm.py`), built to isolate why a streamed trajectory tracks a
commanded position more slowly than a single one-shot move to the same
target. This is the servo-response-lag gap referenced in
`controllers.yaml`'s comment above `left_arm_controller` (MoveIt-issued
trajectories hit "state tolerance violation" against real hardware even
though mock hardware tracks the identical trajectory to within
0.004-0.005 rad; trajectory/goal tolerance was removed there as a
workaround, pending real numbers to re-tighten with).

All of these drive `right_arm_shoulder_pan` (bus2, servo id 1) directly,
with the arm otherwise idle - safe to run standalone, no ROS node needed.
`moveit_tracking_diag.py` (already committed) attacks the same question
from the other end: plans a real trajectory through MoveIt, then sends it
straight to the controller's `FollowJointTrajectory` action and logs the
desired/actual/error feedback at full rate. Use that one when the
question is "what does the real controller see"; use the scripts below
when the question is "what does the servo actually do for this exact
write pattern."

Run `matrix_test.py` first; it consolidates the narrower probes below into
one side-by-side comparison:

- **`matrix_test.py`** - the full comparison matrix: ACC=100 vs ACC=254,
  write rate vs step-per-write, goal-position-only writes (ACC/Speed set
  once) vs rewriting every register every cycle, and one-shot large moves
  to establish real peak velocity. Prints a live setpoint/actual/error
  trace per case.
- **`acc_test.py`** - does the ACC register actually change real-world
  move speed, across one-shot moves at several ACC/Speed combinations.
- **`speed600_test.py`** - isolates the real driver's actual configured
  speed limit (600 steps/s) against a much higher one (2000), same
  streaming pattern.
- **`stream_test.py`** - does streaming 1-step position increments every
  10ms (matching the real driver's write() cadence) reproduce a slow
  crawl, versus sending the whole move as one command.
- **`repeat_test.py`** - same average advance rate as `stream_test.py`,
  but alternating repeat/advance-by-2 instead of advancing by 1 every
  cycle - checks whether re-sending an unchanged setpoint matters.
- **`syncwrite_test.py`** - replicates the real driver's exact
  `SyncWritePosEx` bundling (all 6 right-arm servos in one packet every
  cycle, only one actually moving) to check whether bundling idle servos
  into the same packet slows the moving one down.

No fix has landed from this yet - these are the isolation tools, not a
conclusion. Re-run and compare against the current tolerance-removal
workaround before re-tightening `left_arm_controller`'s trajectory/goal
tolerances.
