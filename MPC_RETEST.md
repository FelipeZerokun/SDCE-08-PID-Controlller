# MPC revision 2: Ubuntu retest

This revision responds to the first Ubuntu log: steering oscillated between -1 and +1,
with 28.8% of recorded time at full steering and a failed lap. PID and LQR code,
shared state estimation, speed controller, and the telemetry protocol are unchanged.

## What changed

- Steering magnitude costs much more (weight 25 instead of 0.5); steering changes
  also cost more (20 instead of 8). Lateral error weight is 6 and heading weight is 1.
- Every predicted command obeys both amplitude [-1,1] and a hard rate bound of
  1.5 normalized steering units per second. For the first command, the allowed
  change is `1.5 * min(elapsed_seconds, 0.1)`; late packets cannot cause large jumps.
- Before predicting new actions, propagate the estimated state for 0.30 seconds
  with the previous command. This is a conservative response allowance chosen in
  stress tests, NOT an identified physical actuator delay or measured simulator lag.
  It approximates the combined effect of sampled telemetry, filtering, and response.
- A slow integral trim adds `0.1 * CTE * min(dt,0.1)` per update to the error reference.
  It is bounded to +/-1 metre, freezes below 1 m/s and near steering saturation, and
  resets on reconnect/reset/pause. This reduces persistent offsets with conservative
  gains; it is an outer correction, not an optimized integral state.
- The terminal cost now uses a continuous-time LQR value matrix for Q=diag(6,1), R=25,
  scaled by the prediction step. It replaces the arbitrary diagonal terminal weights.
  This is a heuristic for the sampled, constrained problem, not a stability proof.

The optimizer still chooses twelve commands over 1.2 seconds and applies only the
first. This is still MPC, with an integral trim; it does not switch to PID or LQR.
The model still assumes a locally straight road and has no waypoint preview.
Coordinate descent keeps the command sequence feasible, but adjacent-command
constraints can prevent it reaching the global optimum. Its 80-sweep limit and
small-coordinate stopping condition do not guarantee optimality.

## Validation and limits

The new `mpc_timing_test` simulates a nonlinear bicycle plant at 15 and 25 mph,
0/100/180 ms pure input delays, an additional 80 ms actuator lag, +/-20% steering
response mismatch, and irregular telemetry. The plant integrates at <=5 ms while
holding each command until the next telemetry sample. Road curvature is hidden
from the controller. Acceptance limits are RMS CTE <0.65 m after startup, maximum
CTE <1.8 m, and <5% steering saturation. These thresholds are experimental checks,
not lane-boundary guarantees.

The original MPC failed 11 of the 12 fixed-timing stress cases. Revision 2 passes
all 12, as well as the unchanged original nine tracking cases. It also passes all
12 cases for each recorded timing sequence (PID, LQR, MPC). These timing-driven runs
use only sample intervals from the CSV; the simulated car responds to the NEW commands.
They do not replay the old CTE as though it were a new closed-loop result.

The tuning used these stress scenarios, so their results are development evidence,
not independent simulator validation. No revised Unity lap has been observed yet.
This revision trades some ideal-model tracking speed for smoother, more tolerant
behavior. It is not claimed to outperform LQR. Start at 15 mph before testing 25 mph.

## On Ubuntu

From the repository root, after obtaining this revision:

```bash
git pull --ff-only
cmake --build --preset ubuntu-simulator
ctest --preset ubuntu-simulator
./build/ubuntu-simulator/mpc_timing_test outputs/mpc-run1.csv
./build/ubuntu-simulator/mpc --controller mpc --speed 15 --log outputs/mpc-v2-15-run1.csv
```

The optional timing test expects the normal telemetry CSV format; it ignores intervals
over 0.5 s, which the controller treats as pauses. The normal CTest case embeds a fixed
irregular timing sequence and does not require any local logs.

If 15 mph completes a lap without growing oscillations, reset/reconnect and run:

```bash
./build/ubuntu-simulator/mpc --controller mpc --speed 25 --log outputs/mpc-v2-25-run1.csv
python3 scripts/summarize_run.py outputs/pid-run1.csv outputs/lqr-run1.csv outputs/mpc-v2-25-run1.csv
```

Preserve the failed original `mpc-run1.csv`. Compare 25 mph runs on the same track,
starting point, and lap count. The 15 mph check is not a fair speed-matched comparison.
If revised MPC still leaves the road, stop the run and retain the log. Better actuator
identification or waypoint telemetry may be needed; numerical tests cannot certify a lap.
