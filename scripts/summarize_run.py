"""Summarize live telemetry logs on Windows/Linux. Standard library only."""
import argparse
import csv
import math
import sys
from collections import defaultdict


def summarize(path):
    groups = defaultdict(list)
    with open(path, newline="", encoding="utf-8-sig") as stream:
        for row in csv.DictReader(stream):
            groups[(row["connection"], row["segment"], row["controller"])].append(row)
    for key, rows in groups.items():
        duration = squared = rate_squared = saturated = compute = speed_sum = maximum = 0.0
        previous = None
        for row in rows:
            dt, e, u, speed, us = (float(row[k]) for k in
                                  ("dt_s", "cte_m", "steering", "speed_mph", "compute_us"))
            if dt <= 0 or not all(map(math.isfinite, (dt, e, u, speed, us))):
                raise ValueError(f"Invalid sample in {path}")
            duration += dt
            squared += e * e * dt
            maximum = max(maximum, abs(e))
            if previous is not None:
                rate_squared += (u - previous) ** 2 / dt
            saturated += dt if abs(u) >= 0.999 else 0
            compute += us
            speed_sum += speed * dt
            previous = u
        yield [path, *key, duration, math.sqrt(squared / duration), maximum,
               math.sqrt(rate_squared / duration), 100 * saturated / duration,
               speed_sum / duration, compute / len(rows)]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+")
    args = parser.parse_args()
    writer = csv.writer(sys.stdout)
    writer.writerow(["file", "connection", "segment", "controller", "duration_s",
                     "rms_cte_m", "max_cte_m", "rms_steering_rate_per_s", "saturation_percent",
                     "mean_speed_mph", "mean_compute_us"])
    for path in args.logs:
        writer.writerows(summarize(path))
