# Visualize simulator runs

`plot_runs.py` compares actual simulator logs on Ubuntu and Windows. It is separate
from `plot_comparison.py`, which plots the synthetic benchmark's different CSV format.

On Ubuntu, install plotting support once:

```bash
sudo apt install python3-matplotlib
```

From the project root:

```bash
python3 scripts/plot_runs.py \
  outputs/pid-run1.csv outputs/lqr-run1.csv outputs/mpc-v2-25-run1.csv \
  --labels "PID 25 mph" "LQR 25 mph" "MPC v2 25 mph" \
  --out outputs/plots-25
```

Open `outputs/plots-25/metrics.png` and `timeseries.png` with your image viewer.
The script also writes `summary.csv` and a text explanation. It saves files without
requiring a graphical desktop. Existing named outputs are refused; choose a new
`--out` folder for subsequent reports. Input logs are never modified.

The same script works with `python` on Windows when matplotlib is installed.
For PowerShell, put the command on one line rather than using Bash backslashes.

## Higher speeds and repeat runs

After collecting the three 30 mph logs:

```bash
python3 scripts/plot_runs.py \
  outputs/pid-30-run1.csv outputs/lqr-30-run1.csv outputs/mpc-v2-30-run1.csv \
  --labels "PID 30 mph" "LQR 30 mph" "MPC v2 30 mph" \
  --out outputs/plots-30
```

You can also pass several runs of one controller to examine its response to speed:

```bash
python3 scripts/plot_runs.py \
  outputs/mpc-v2-15-run1.csv outputs/mpc-v2-25-run1.csv outputs/mpc-v2-30-run1.csv \
  --labels "MPC 15 mph" "MPC 25 mph" "MPC 30 mph" \
  --out outputs/mpc-speed-plots
```

Provide only files that exist. Each file remains a separate run; repeated runs are
not silently pooled into a single result. Labels are optional (default: filenames),
and are for display only: target speed and lap success are not present in the logs.
Keep labels short for readable figures.

## Reading the charts

- Time-series: tracking error, commanded steering, actual speed, and compute time.
- Metrics: RMS/max error, steering-rate RMS, time at full steering, mean speed, and
  mean compute time. All rows contribute; error/speed/saturation use time weighting.
- Summary CSV also includes duration, segment count, sample count, and median update
  interval. Compute time is averaged per update, not weighted by elapsed time.

Time-series lines break across connection/reset/pause segments. The x-axis accumulates
logged dt and excludes omitted pauses; it does NOT align vehicles by track position.
Steering-rate metrics skip segment boundaries and describe commands, not measured
vehicle steering. There is no automatic lap-completion detection or winner ranking.
Compare runs with matching target speed, starting point, track and lap count, and
record failed laps separately. Mixed-speed charts are speed experiments, not equal
conditions for ranking controllers.

Metric tests (Python standard library, no matplotlib required):

```bash
python3 -m unittest discover -s tests -p test_plot_runs.py
```
