# A small MPC learning experiment

For the PID/LQR/MPC comparison, see [CONTROLLERS.md](CONTROLLERS.md).

The root project now runs a lateral model predictive controller with the original
PID simulator telemetry. Your historical solution remains under `legacy/`.
The old root PID scaffold is retained but unused; the new PID and LQR live in Controller.cpp.

## Run the numerical checks now (Windows, no simulator dependencies)

```powershell
cmake -S . -B build -DBUILD_SIMULATOR=OFF
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
.\build\Release\mpc_test.exe
```

The executable prints tracking metrics for nine cases: three speeds and three
road curvatures. Its plant uses nonlinear trigonometry and a different wheelbase
from the controller. These are numerical checks, not validation of a Unity lap.

## Run with the Udacity simulator

The bridge uses Boost.Beast and nlohmann-json, following the modern adapter in
`SDCE-05-Extended-Kalman-Filters`. No legacy uWebSockets installation is needed.
The transport handles text WebSockets, Engine.IO ping/pong, Socket.IO namespace
connection, telemetry, reset, malformed-input fallback, and reconnects.

On this machine, reuse the EKF project's already installed dependencies:

```powershell
cmake -S . -B build -DBUILD_SIMULATOR=ON -DCMAKE_PREFIX_PATH="../SDCE-05-Extended-Kalman-Filters/build/windows/vcpkg_installed/x64-windows"
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
.\build\Release\mpc.exe
```

For a standalone checkout, the included vcpkg manifest declares the dependencies.
Configure a fresh build directory using your vcpkg toolchain:

```powershell
cmake -S . -B build-vcpkg -DBUILD_SIMULATOR=ON -DCMAKE_TOOLCHAIN_FILE="$env:VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"
cmake --build build-vcpkg --config Release
```

Open the simulator's PID driving mode. The server binds only to 127.0.0.1:4567.
An optional port argument is supported, e.g. `mpc.exe 4568`, for clients configured
to use that port. The simulator normally expects 4567; close other project servers
using it. Start with normal simulation speed. The target speed is 25 mph in
`src/simulator_session.cpp`.

The Windows server build, protocol checks, and real loopback WebSocket exchanges
are verified. Driving a full lap in the Unity simulator remains a separate check.
The tests include numeric/string telemetry, malformed inputs, ping/pong, and
fresh controller state on reconnect. The EKF project itself is not modified.

## Read the algorithm in this order

1. `src/simulator_session.cpp`: receive CTE and speed; convert mph to m/s; measure elapsed time.
2. `MPC::Update` in `src/MPC.cpp`: estimate heading error from filtered CTE velocity.
3. `MPC::Cost`: predict the effect of twelve future steering commands.
4. The coordinate-minimization loop: improve that sequence subject to bounds.
5. Return `plan_[0]`: apply one command, measure again, and solve again.

This last step is **receding horizon control**, the defining MPC feedback loop.
A prediction is only a hypothesis; fresh measurements correct it at every update.

## Model and assumptions

State: lateral error `e` in metres and estimated heading error `psi` in radians.
Control: normalized steering `u` in [-1, 1]. Speed `v` is held constant within each
prediction. Each prediction step is `h = 0.1 s`, with 12 steps (1.2 seconds).

```text
r        = (v / L) * delta_max * u
 e_next  = e + v * psi * h + 0.5 * v * r * h^2
 psi_next = psi + r * h
```

This is a small-angle bicycle model with a locally straight reference road.
The effective wheelbase `L = 2.67 m` and steering scale `delta_max = 25 degrees`
are starting assumptions, not parameters measured from your installed simulator.
Positive steering is assumed to increase positive CTE, consistent with the
negative feedback sign in the legacy PID. Verify the response at low speed.

Only CTE and speed are used. There is no heading sensor or path preview in this
bridge. Heading is approximated as filtered `(change in CTE / elapsed time) / v`,
limited to +/-0.5 radians; the first sample and low-speed operation use zero.
The derivative filter has a time constant of 0.08 seconds. Pauses above 0.5 seconds
and reconnects reset controller history. Restart/reconnect after manually resetting
the car so a position jump is not interpreted as a real lateral velocity.

## What the optimizer tries to minimize

```text
sum over predicted steps:
    5 * e^2 + 2 * psi^2 + 0.5 * u^2 + 8 * (u - previous_u)^2
plus terminal cost:
    10 * final_e^2 + 5 * final_psi^2
```

Tracking error matters, but large steering and sudden changes also have a cost.
Steering magnitude is a hard constraint. Steering changes are penalized, not
hard-limited. There are no lane-boundary constraints in this first version.

The model is linear and the cost quadratic: this is a small convex quadratic
program. We minimize one command at a time while holding the others fixed, using
three cost evaluations to recover that coordinate's quadratic minimum, then
clipping to [-1, 1]. Repeat for at most 80 sweeps. This is an approximate solver
with a bounded iteration count, not a guarantee of an optimal or stabilizing
solution. The previous plan is resampled to warm-start the next solve.

Throttle uses a separate proportional speed controller capped at +/-0.3. This MPC
does not jointly optimize steering and acceleration or model actuator delay.

## Small experiments

- Increase the CTE weight from 5 to 10: observe more aggressive error correction.
- Increase the steering-change weight from 8 to 20: observe the smoothness/tracking tradeoff.
- Change the horizon from 12 to 6: compare shorter prediction and computation time.
- After validating a lap, gradually change the target speed and record error.

Change one parameter at a time. Run the numerical checks and simulator again.
The small-angle model and heading estimate become less reliable in sharp turns,
at large errors, and with noisy or irregular telemetry. Constant-curvature tests
cannot establish performance on the real track.

## Which controller might work better?

A tuned PID may match or outperform this limited-input MPC on the original track.
LQR is a useful next comparison: model-based feedback with much less online work,
but ordinary LQR does not explicitly enforce actuator bounds. Pure Pursuit or
Stanley become attractive when path geometry and heading are available.
For a more capable MPC, the next useful change is waypoint telemetry, measured
pose, curvature preview, and an actuator-delay model, followed by a dedicated QP
solver and measured comparisons. Complexity alone does not improve control.

Background: https://www.do-mpc.com/en/latest/theory_mpc.html
Udacity waypoint interface: https://github.com/udacity/CarND-MPC-Project/blob/master/src/main.cpp
