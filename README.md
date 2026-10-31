# PID, LQR, and MPC controller comparison

A modern extension of the original Udacity Self-Driving Car Engineer PID project.
Compare three lateral controllers using the same telemetry, speed control, and
logging, with a Boost.Beast simulator connection and a synthetic vehicle benchmark.

- [Simulator results and conclusions](RESULTS.md)
- [Controller comparison and experiments](CONTROLLERS.md)
- [Plot simulator logs and speed comparisons](VISUALIZATION.md)
- [Windows and Ubuntu 22.04.5 setup](PLATFORM_SETUP.md)
- [MPC model, objective, and implementation](MPC_GUIDE.md)
- [Revised MPC and Ubuntu retest](MPC_RETEST.md)
- [Revised PID/LQR and timing-aware retest](PID_LQR_RETEST.md)

## Quick offline build

Windows (CMake and a C++17 compiler installed):

```powershell
cmake --preset windows-offline
cmake --build --preset windows-offline
ctest --preset windows-offline
.\build\windows-offline\Release\compare_controllers.exe outputs/comparison-1
```

Ubuntu (CMake, Ninja, and a C++17 compiler installed):

```bash
cmake --preset ubuntu-offline
cmake --build --preset ubuntu-offline
ctest --preset ubuntu-offline
./build/ubuntu-offline/compare_controllers outputs/comparison-1
```

See the platform guide for the vcpkg-based simulator build. Run one server at a time
with `--controller pid`, `--controller lqr`, or `--controller mpc`. The server binds
to localhost port 4567; use the simulator's PID driving mode on the same machine.

## What is included

`src/Controller.cpp` implements the new PID and LQR baselines; `src/MPC.cpp`
implements the predictive controller. Shared estimation, CSV metrics, protocol
handling, tests, and analysis scripts support repeatable comparisons.
The new PID uses time-aware integration/derivatives and is not an exact replay of
the original tuned PID solution.

Windows builds and five numerical/protocol test suites passed before the latest controller release. The user verified Ubuntu builds/tests and simulator laps. PID v2 and MPC v2 completed the higher-speed experiments; LQR v2 left the track in its 50 mph-target run, recovered, and finished. Those runs reached only about 34 mph. See [RESULTS.md](RESULTS.md) for measured results, timing limitations, and reported outcomes. Synthetic benchmarks are separate from simulator validation.

## Original project and local files

The original submission remains in the repository's `main` branch and history.
This comparison branch replaces the old nested project with the maintained project
at the repository root. Local `legacy/`, `Simulator/`, and `simulators/` copies,
build directories, and generated outputs are ignored and are not distributed.
The local legacy copy is optional and is not needed to build this branch.

The modern transport follows the adapter developed in the author's
SDCE-05-Extended-Kalman-Filters project. No legacy uWebSockets installation is needed.
