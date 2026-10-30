# PID, LQR, and MPC: run and understand the comparison

For Ubuntu 22.04.5 commands, see [PLATFORM_SETUP.md](PLATFORM_SETUP.md).
The examples below use Windows paths; all controllers also have Ubuntu build presets.

## Try all three in the Udacity simulator

Build as described in MPC_GUIDE.md, then run ONE server at a time from this folder:

```powershell
.\build\Release\mpc.exe --controller pid --speed 25 --log outputs/pid-25-run1.csv
.\build\Release\mpc.exe --controller lqr --speed 25 --log outputs/lqr-25-run1.csv
.\build\Release\mpc.exe --controller mpc --speed 25 --log outputs/mpc-25-run1.csv
```

For each command: start/reset the simulator in PID mode, drive the same track from
the same starting point, and stop after the same number of laps or a fixed duration.
Disconnect and stop the server with Ctrl+C before starting the next one. Repeat each
controller three times. Use a fresh log filename each time; existing logs are refused.
The executable keeps its historical `mpc` name but selects any of the three controllers.
All three use exactly the same proportional throttle control, target speed, telemetry,
steering bounds, and derivative filter. Actual speed can differ; compare mean speed too.
The server prints a summary on disconnect. Logs are flushed on every telemetry sample.

```powershell
.\scripts\Summarize-Run.ps1 -Path outputs/pid-25-run1.csv,outputs/lqr-25-run1.csv,outputs/mpc-25-run1.csv | Format-Table
```

Each connection, explicit reset, error, or long pause separates measurement segments.
Reconnect when manually resetting the car if the simulator doesn't emit a reset event.
Wall-clock timing assumes normal simulation speed; there is no simulator timestamp.
Mark crashes and incomplete laps separately: a short failed run is not a successful
low-error result. The current telemetry cannot automatically identify completed laps.

## Run the repeatable synthetic benchmark

```powershell
.\build\Release\compare_controllers.exe outputs/comparison-run2
```

Choose a NEW output directory. This requires no Unity simulator or WebSocket libraries.
It runs 36 cases: 3 controllers x 3 speeds (15,25,40 mph) x 4 scenarios, 20 seconds each.
Each controller sees CTE and speed only, starting 1 m off the reference with zero heading
error. The plant uses nonlinear sine/tangent terms and a 2.8 m wheelbase, while the
controllers assume 2.67 m. This remains a simplified lateral plant, not a full car.

- `straight`: recovery from the starting offset.
- `curves`: sinusoidal road curvature after 3 seconds; no controller gets preview.
- `bias`: a constant +0.025 normalized steering bias.
- `delay`: the curves scenario with a five-sample (100 ms) command delay.

The CSV summary includes time-weighted RMS CTE, maximum absolute CTE, RMS commanded
steering rate, time at steering saturation, and mean controller computation time.
Every case also gets a time-series CSV. Timing excludes logging/network/plant updates;
absolute microsecond measurements depend on the machine and build configuration.
Initial recovery is INCLUDED in all metrics. Lower RMS steering rate generally means
smoother commands, but must be assessed together with tracking and run completion.

To plot the 25 mph curves and delay cases (requires Python with matplotlib):

```powershell
python scripts/plot_comparison.py outputs/comparison-run2
```

The first run's plot is in `outputs/comparison-initial/comparison.png`. It shows the
MPC oscillating and repeatedly saturating under unmodeled delay, even though it tracks
well without delay. Inspect steering as well as error before judging performance.

## What each controller actually does

| | PID | LQR | MPC |
|---|---|---|---|
| Steering rule | Weighted error, accumulated error, and error rate | State feedback `u = -K x` | Optimize a sequence, apply the first command, repeat |
| Vehicle model | Not required | Linear model used to derive gains | Model used in every prediction |
| Optimization | No optimization at runtime | Infinite-horizon quadratic problem solved when deriving gains | Finite-horizon constrained problem solved repeatedly |
| Bounds here | Clip output, prevent integral windup | Clip output AFTER computing the unconstrained solution | Include steering bounds INSIDE the optimization |
| Memory here | Integral and derivative filter | State estimator/filter | State estimator/filter and previous optimized plan |
| Main weakness here | Gains depend on operating conditions | Model assumptions; clipping loses unconstrained optimality | Computation, model mismatch, and unmodeled delay |

### PID: correct the error directly

`u = -(Kp * e + Ki * integral(e dt) + Kd * filtered(de/dt))`

P pulls the car toward the reference. D damps motion away from or toward it. I builds
up a correction for sustained bias. We use `Kp=0.12, Ki=0.015, Kd=0.18`, explicit
seconds, an 0.08 s derivative filter, a bounded integral, and conditional integration
when saturated. This is a new baseline, NOT an exact replay of the six-year-old PID:
its per-message gains cannot be reused directly with a time-scaled implementation.
Read the PID branch in `src/Controller.cpp`.

### LQR: derive feedback gains from a model and a cost

State `x=[e,psi]` contains lateral and heading error. The continuous linear model is:

```text
e_dot   = v * psi
psi_dot = (v / L) * delta_max * u
J       = integral (e^2 + psi^2 + 25*u^2) dt, from now to infinity
u       = -K_e*e - K_psi*psi
```

For this two-state model, the stabilizing algebraic Riccati equation has a closed-form
solution: `K_e=0.2`, `K_psi=sqrt((10*L/delta_max+1)/25)`, approximately 1.577.
Both dynamics scale with positive speed, so these continuous-time gains are constant.
There is no need for an online matrix solver. At standstill the car cannot correct
lateral error; the code uses the positive-speed law as a low-speed extension.

LQR accounts for future consequences through its model-derived gains even though it
does not roll out a trajectory on every update. Its optimality applies to the specified
linear, unconstrained problem; real-car mismatch, sampled updates, and output clipping
change that problem. This LQR has no integral state or curvature feedforward, so a
sustained bend or steering bias can leave a nonzero offset.
Read the LQR branch and derivation in `src/Controller.cpp`.

### MPC: choose the next sequence under constraints

Our MPC predicts 12 steps of 0.1 seconds. It penalizes lateral/heading error, steering
magnitude, and changes in steering. It constrains normalized steering to [-1,1], solves
an approximate quadratic optimization by coordinate minimization, applies the first
command, and repeats with fresh measurements. See `src/MPC.cpp` and MPC_GUIDE.md.
The horizon is finite, and its terminal penalty is a chosen weight, not an LQR terminal
Riccati matrix. The solver's sweep budget is not a convergence guarantee.

MPC and LQR are related: with compatible linear dynamics, quadratic costs, no active
constraints, and suitable terminal treatment, MPC can reproduce LQR feedback. Our
implementations use different costs and horizons, so this comparison is not a test of
that equivalence. MPC's extra flexibility pays off only when its model and objective
capture what matters. Here it has NO preview or actuator-delay compensation.

## Initial results: an experiment, not a universal ranking

The first Release benchmark produced these RMS errors at 25 mph (metres):

| Scenario | PID | LQR | MPC |
|---|---:|---:|---:|
| Straight recovery | 0.209 | 0.158 | 0.116 |
| Curves | 0.525 | 0.346 | 0.150 |
| Steering bias | 0.224 | 0.206 | 0.124 |
| Curves + 100 ms delay | 0.530 | 0.351 | 0.474 |

MPC tracks best in the no-delay cases with these settings. LQR has less error in the
delayed case. At 40 mph, delayed-case RMS rises to 1.607 m for MPC, versus 0.346 m for
LQR and 0.832 m for PID. This is evidence of sensitivity to unmodeled delay, not proof
that one controller family is inherently better. The finite test also includes the
initial transient, so it does not isolate steady-state bias rejection by PID's integral.

No equally intensive tuning, road-boundary checks, noise study, or Unity lap validation
has been performed. These are controller-plus-tuning comparisons. First compare their
behavior; then change ONE weight/gain at a time and rerun identical scenarios.

Suggested learning order: PID P/I/D changes; LQR change R (steering effort); MPC change
the steering-change penalty or horizon; finally model the actuator delay in MPC and
repeat the delayed case. A waypoint-based MPC would be a separate next extension.

Sources: [MIT LQR notes](https://underactuated.mit.edu/lqr.html) and
[do-mpc theory](https://www.do-mpc.com/en/latest/theory_mpc.html).
