# PID, LQR, and MPC controller comparison

A modern extension of the original Udacity Self-Driving Car Engineer PID project.
Compare three lateral controllers using the same telemetry, speed control, and
logging, with a Boost.Beast simulator connection and a synthetic vehicle benchmark.

- [Controller comparison and experiments](CONTROLLERS.md)
- [Windows and Ubuntu 22.04.5 setup](PLATFORM_SETUP.md)
- [MPC model, objective, and implementation](MPC_GUIDE.md)
- [Revised MPC and Ubuntu retest](MPC_RETEST.md)

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

Windows builds and numerical/protocol tests have been verified. The user also verified the Ubuntu build/tests and a PID lap. The first MPC failed its lap; revision 2 needs a new Ubuntu driving test. Synthetic benchmark results are not real simulator lap results.

## Original project and local files

The original submission remains in the repository's `main` branch and history.
This comparison branch replaces the old nested project with the maintained project
at the repository root. Local `legacy/`, `Simulator/`, and `simulators/` copies,
build directories, and generated outputs are ignored and are not distributed.
The local legacy copy is optional and is not needed to build this branch.

The modern transport follows the adapter developed in the author's
SDCE-05-Extended-Kalman-Filters project. No legacy uWebSockets installation is needed.
