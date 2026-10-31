# PID and LQR revision 2: retest on Ubuntu

MPC v2, the shared CTE derivative estimator, throttle controller, and simulator
protocol are unchanged. This revision modifies only the PID/LQR control paths and
adds an independently integrated nonlinear stress test.

## Simulator results now available

The 30 and 50 mph-target runs are documented in [RESULTS.md](RESULTS.md). PID v2 completed both with no departure reported. LQR v2 completed both, but left the track and recovered in the 50 mph-target run. Actual speeds peaked around 34 mph in the latter tests. The commands below remain a repeat-test procedure.

## Why a slow laptop matters

We measure elapsed time with the server's wall clock. If Unity stalls, messages can
arrive less frequently or irregularly. If simulation time advances more slowly than
wall time, `change in CTE / wall dt` no longer estimates the simulated lateral velocity
correctly. Neither effect is established by CPU usage alone. The current telemetry
contains no simulator timestamp, so the code does NOT pretend to recover simulation
time or silently substitute a fixed dt.

The existing logs showed different median intervals across controllers, even though
PID/LQR computation was cheaper than MPC. This is consistent with different runtime
conditions, but is not proof of a particular graphics/CPU bottleneck.

For retests, use the same simulator resolution/quality settings and normal simulation
speed, close unnecessary background applications, run one server at a time, and avoid
changing settings between controllers. If reducing graphics quality helps, repeat ALL
controllers under those settings rather than comparing against the old stressed run.

## PID changes

The `pid` selection now uses a predictive PID wrapper, NOT a purely model-free PID:

- Gains: Kp=0.25, Ki=0.003, Kd=0.12*min(1,11.2/v), with v in metres/second.
- The existing filtered error derivative is retained. D gain decreases above 11.2 m/s.
- A first-order 0.15 s response estimate filters the previous command.
- A bicycle-model estimate of lateral acceleration predicts error/error rate 0.35 s
  ahead under that estimated held command. P and D act on those predicted values.
- The integral uses measured CTE, is bounded to +/-10, freezes below 1 m/s, and its
  update interval is capped at 0.1 s. Conditional integration considers both amplitude
  saturation and steering-rate limiting.
- Output is limited to [-1,1] and changes by at most 1.5*min(dt,0.1) per update.

The response allowances are tuning assumptions, not measured physical delays.
The extra model assumption makes this a comparison of engineered implementations,
not a clean experiment isolating a model-free PID against model-based algorithms.
The original PID is still available in Git history and your existing baseline logs.

## LQR changes

`src/LQR.cpp` solves the discrete algebraic Riccati equation for:

```text
x = [lateral error, heading error]
A = [[1, v*h], [0, 1]]
B = [[0.5*v*b*h*h], [b*h]], b = v*delta_max/L
Q = diag(3,1), R = 25
K = (R + B' P B)^-1 B' P A
```

The model step h follows a smoothed telemetry interval bounded to 0.05..0.20 s,
starting at 0.1 s. Speed is floored at 1 m/s when calculating gains because the
standstill model cannot steer away lateral error. This is a low-speed extension,
not a claim of standstill controllability. Riccati iteration checks convergence;
it throws rather than using unconverged gains.

The state is propagated through a 0.30 s response allowance using the same filtered
command-response estimate as PID, then feedback `u=-K*x_predicted` is applied.
There is no integral state in this LQR. The amplitude and rate limits are applied
AFTER the LQR calculation, so constrained optimality is not claimed. Discrete-time
model feedback does not guarantee robustness to arbitrary delays or clock mismatch.

## Numerical checks

`controller_timing_test` runs 24 cases: PID/LQR, 25/30 mph, 0/100/180 ms command delay,
and +/-20% steering response mismatch. An independent nonlinear plant has a different
wheelbase and an additional 80 ms actuator lag. It integrates in steps no larger than
5 ms while commands are held between telemetry updates. The controller receives the
PREVIOUS interval that produced the current measurement, not the next planned interval.

The revised controllers pass all fixed-pattern cases, plus cases using the recorded
30 mph timing sequences. Acceptance thresholds: post-startup RMS CTE below 0.9 m,
maximum error below 2 m, and less than 5% steering saturation. These are development
checks, not road-edge or lap-completion guarantees. Tuning used these tests. Subsequent
Ubuntu simulator results are recorded separately in [RESULTS.md](RESULTS.md).

An optional clock-scale argument explores simulation-time mismatch. With a 0.5 scale
and the PID 30 mph intervals, the test remains bounded but six cases exceed the 0.9 m
RMS threshold (worst approximately 0.94 m). This explicitly demonstrates that the
revision does not solve unknown simulator-time scaling. Do not claim all slowdown
scenarios pass or assume lag compensation is a substitute for consistent performance.

Additional tests cover rate limits, resets, symmetry, invalid inputs, nominal discrete
LQR closed-loop stability, and the small-step limit toward continuous-time gains.
The two-state solver is run each LQR update; its computation cost differs from the old
constant-gain LQR. Measured computation times are included in [RESULTS.md](RESULTS.md).

## Ubuntu commands

```bash
git pull --ff-only
cmake --build --preset ubuntu-simulator
ctest --preset ubuntu-simulator
./build/ubuntu-simulator/controller_timing_test outputs/pid-30-run1.csv
./build/ubuntu-simulator/controller_timing_test outputs/lqr-30-run1.csv
```

Optional stress exploration (may report FAIL for the clock-mismatch cases above):

```bash
./build/ubuntu-simulator/controller_timing_test outputs/pid-30-run1.csv 0.5
```

First run at 25 mph, resetting the simulator and stopping the server between commands:

```bash
./build/ubuntu-simulator/mpc --controller pid --speed 25 --log outputs/pid-v2-25-run1.csv
./build/ubuntu-simulator/mpc --controller lqr --speed 25 --log outputs/lqr-v2-25-run1.csv
```

If each completes with controlled steering, repeat at 30 mph:

```bash
./build/ubuntu-simulator/mpc --controller pid --speed 30 --log outputs/pid-v2-30-run1.csv
./build/ubuntu-simulator/mpc --controller lqr --speed 30 --log outputs/lqr-v2-30-run1.csv
```

Keep old files. Record whether each lap completed and whether oscillation grew. If
computer performance/settings differ from earlier tests, rerun the unchanged MPC
under the same conditions for a meaningful comparison. Do not jump to 50 mph based
on numerical checks at 25/30 mph.

Plot a matched-speed comparison:

```bash
python3 scripts/plot_runs.py outputs/pid-v2-30-run1.csv outputs/lqr-v2-30-run1.csv outputs/mpc-v2-30-run1.csv --labels "PID v2 30" "LQR v2 30" "MPC v2 30" --out outputs/revision2-30-plots
```

Read the [MIT LQR notes](https://underactuated.mit.edu/lqr.html) for the distinction
between continuous and discrete Riccati feedback and its model assumptions.
