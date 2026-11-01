# Diagnostic simulator logging

Diagnostic schema 1 appends columns to every new `--log` CSV. The first ten columns
retain their names, order, and meaning. Existing plot/summarize scripts read named
columns and accept the additions. Older CSVs remain supported. No extra flag is needed.
This revision adds observation only: gains, prediction, optimization, command limits,
and throttle control are unchanged. It cannot add diagnostics to old recordings.

## Capture on Ubuntu

```bash
git pull --ff-only
cmake --build --preset ubuntu-simulator
ctest --preset ubuntu-simulator
./build/ubuntu-simulator/mpc --controller lqr --speed 30 --log outputs/lqr-v2-diag-30-run1.csv
```

Reset the simulator between runs and use a fresh filename. After the initial check,
repeat the previously problematic scenario with `--speed 50` and a filename such as
`lqr-v2-diag-50-run1.csv`. This is a target, not a promise of actual speed. Capture
PID and MPC reference runs under the same graphics/load conditions. Note lap completion,
track departures, recovery, and pauses separately. Preserve the whole recording.
Record `git rev-parse HEAD` with your run notes; the CSV does not embed a binary revision.

## Additional fields

Blank means unavailable or inapplicable, never zero. Steering values are normalized
commands in [-1,1], except raw demands, which may exceed that range.

| Field | Meaning |
| --- | --- |
| `diagnostic_schema` | 1 for this appended schema |
| `wall_dt_s` | Actual interval supplied to the controller, including pauses; first sample after reset uses 0.02 s |
| `target_mph` | Configured target speed |
| `throttle_raw` | Proportional throttle demand before clipping to +/-0.3 |
| `pause_reset` | 1 when wall dt exceeds 0.5 s and triggers a pause reset |
| `cte_rate_mps` | Filtered derivative of measured CTE using wall-clock time |
| `heading_est_rad` | Estimated heading error used by the controller; bounded +/-0.5 rad, zero at speed <=1 m/s |
| `previous_command` | Previous issued command used this update; zero following reset |
| `response_command_est` | PID/LQR filtered command-response estimate; not measured steering |
| `predicted_cte_m` | Error after the controller's assumed response allowance; MPC includes integral bias |
| `predicted_heading_rad` | LQR/MPC heading after their response allowance; blank for PID |
| `predicted_cte_rate_mps` | PID predicted error derivative; blank for LQR/MPC |
| `raw_steering` | PID/LQR feedback demand before amplitude/rate clipping, after PID anti-windup decision; blank for MPC |
| `steering_lower`, `steering_upper` | Intersection of amplitude and previous-command rate bounds for the first action |
| `pid_integral_m_s` | PID integral state after the conditional integration decision |
| `pid_kp`, `pid_ki`, `pid_kd` | PID gains used for this update |
| `lqr_error_gain`, `lqr_heading_gain` | Discrete LQR feedback gains used for this update |
| `model_step_s` | LQR's scheduled model interval or MPC's fixed horizon step; blank for PID |
| `mpc_bias_m` | MPC's bounded integral error trim |
| `mpc_sweeps` | Number of coordinate-descent sweeps, 1..80 |
| `mpc_last_change` | Largest coordinate change in final sweep, in normalized steering units |

The existing `steering` field remains the command sent, not measured actuator position.
MPC has no separate unconstrained raw command: its optimization itself enforces limits.
Its logged bounds describe the first-action amplitude/rate interval, not all constraints
coupling the optimized sequence. A small last change does not prove optimality.

Existing `dt_s` still replaces intervals over 0.5 s with 0.02 s and starts a new segment
for metrics. `wall_dt_s` preserves the original pause interval for investigation.
A controller reset clears diagnostic snapshots and state. Subsequent valid rows reflect
the new state. Explicit resets/errors/reconnects retain existing segment/connection
behavior; `pause_reset` identifies long-interval resets only, not every reset reason.

## Investigating the LQR departure

Inspect several seconds before and after the error grows, not only the largest CTE:

1. Compare `wall_dt_s`, CTE rate, and heading estimate. Packet gaps or the heading
   estimate hitting +/-0.5 may explain a misleading feedback state.
2. Compare measured and predicted error/heading. Prediction depends on an assumed
   0.30 s allowance and estimated response, not an identified actuator delay.
3. Compare `raw_steering` to `steering` and the bounds. A raw demand outside the
   interval demonstrates command clipping. It does not prove increasing the limit
   would improve tracking. The amplitude-only interval is [-1,1].
4. Check gains and `model_step_s` for variation around that event.
5. Compare actual speed, target, and raw throttle to separate speed demand from the
   achieved motion. Then compare repeat runs and the reference controllers.

Diagnostics are read-only snapshots. Extra serialization and the existing per-sample
flush can add runtime overhead, so compare intervals in new runs too. `compute_us`
includes controller computation/snapshot work but excludes CSV serialization, flush,
network transfer, and Unity rendering. There is still no simulator clock, measured
steering response, track-position alignment, or automatic lap/departure detector.
