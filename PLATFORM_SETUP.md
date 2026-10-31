# Windows and Ubuntu 22.04.5 LTS (Jammy)

The same C++17 sources implement PID, LQR, MPC, the synthetic benchmark, and the
Boost.Beast simulator server on both platforms. Use Windows for development/offline
experiments and Ubuntu for the Linux Unity simulator. There is no uWebSockets dependency.

Validation: Windows Release builds, controller tests, and loopback protocol tests pass.
The Windows offline preset also passes. The user verified builds/tests and Unity
driving on a separate Ubuntu 22.04.5 laptop. These are user-run Ubuntu checks, not
local Windows execution of Unity. See [RESULTS.md](RESULTS.md) for lap outcomes.

## Ubuntu: prerequisites

Jammy's CMake 3.22 and GCC 11 meet the project requirements.

```bash
sudo apt update
sudo apt install build-essential cmake ninja-build git curl zip unzip tar pkg-config python3
```

Run subsequent build commands in this project's root. Do not copy Windows build
artifacts or vcpkg libraries to Ubuntu: each platform must compile its own binaries.
The presets use separate build subdirectories to avoid sharing CMake caches.

## Ubuntu: numerical experiments, no simulator dependencies

```bash
cmake --preset ubuntu-offline
cmake --build --preset ubuntu-offline
ctest --preset ubuntu-offline
./build/ubuntu-offline/compare_controllers outputs/ubuntu-comparison-1
```

The benchmark output directory must be new. This configuration builds controller tests
and the comparison program, but not the server or WebSocket tests.

## Ubuntu: server and Unity simulator

Reuse the Linux vcpkg checkout you used for EKF by setting VCPKG_ROOT to its location.
If you do not already have one, install it once:

```bash
git clone https://github.com/microsoft/vcpkg.git "$HOME/vcpkg"
"$HOME/vcpkg/bootstrap-vcpkg.sh"
```

Then, adjusting the path if your existing checkout is elsewhere:

```bash
export VCPKG_ROOT="$HOME/vcpkg"
cmake --preset ubuntu-simulator
cmake --build --preset ubuntu-simulator
ctest --preset ubuntu-simulator
./build/ubuntu-simulator/mpc --controller pid --speed 25 --log outputs/ubuntu-pid-1.csv
```

CMake installs the declared Boost.Beast and nlohmann-json dependencies through vcpkg.
The manifest pins dependency versions. Initial configuration requires internet access
and can take time to compile dependencies. Reuse your EKF vcpkg setup if available.

Start the LINUX Term 2 simulator on the same Ubuntu machine and select PID mode.
A Windows .exe cannot be used as the native Ubuntu simulator. Use your working Linux
simulator from EKF if it provides PID mode. The simulator binary is separate from this
project; it still needs a functioning graphical desktop/graphics environment.

Stop the server and reset/reconnect the simulator between runs. Select LQR or MPC:

```bash
./build/ubuntu-simulator/mpc --controller lqr --speed 25 --log outputs/ubuntu-lqr-1.csv
./build/ubuntu-simulator/mpc --controller mpc --speed 25 --log outputs/ubuntu-mpc-1.csv
```

Run ONE server at a time. It binds to 127.0.0.1:4567, so close the EKF server first.
This setup expects simulator and controller on the SAME machine; it does not configure
a network connection from Ubuntu to Windows. Existing log filenames are refused.

Summarize logs with the portable Python standard-library script:

```bash
python3 scripts/summarize_run.py outputs/ubuntu-pid-1.csv outputs/ubuntu-lqr-1.csv outputs/ubuntu-mpc-1.csv
```

Optional synthetic benchmark plots:

```bash
sudo apt install python3-matplotlib
python3 scripts/plot_comparison.py outputs/ubuntu-comparison-1
```

## Windows

The existing `build/Release` setup remains usable. The new independent offline preset:

```powershell
cmake --preset windows-offline
cmake --build --preset windows-offline
ctest --preset windows-offline
.\build\windows-offline\Release\compare_controllers.exe outputs/windows-comparison-1
```

For a standalone Windows server build using your existing vcpkg installation:

```powershell
$env:VCPKG_ROOT = 'C:/Users/epile/tools/vcpkg'
cmake --preset windows-simulator
cmake --build --preset windows-simulator
ctest --preset windows-simulator
```

Windows presets let CMake select the installed default compiler/generator. A Visual
Studio build places executables in Release; the Ubuntu Ninja build has no Release
subdirectory. Server availability on Windows does not fix the Unity simulator issue.

Reference: [Jammy CMake package](https://packages.ubuntu.com/jammy/amd64/cmake),
[Jammy GCC 11](https://packages.ubuntu.com/jammy/g%2B%2B-11), and
[vcpkg prerequisites](https://learn.microsoft.com/en-us/vcpkg/concepts/supported-hosts).
