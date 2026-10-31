"""Compare real simulator telemetry logs. Requires matplotlib only for plotting."""
import argparse
import csv
import math
from pathlib import Path
import statistics

FIELDS = ("dt_s", "cte_m", "speed_mph", "steering", "compute_us")


def read_run(path, label=None):
    path = Path(path)
    rows = []
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        required = set(FIELDS) | {"connection", "segment", "controller"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"{path}: expected a simulator log, missing {sorted(required-set(reader.fieldnames or []))}")
        for line, row in enumerate(reader, 2):
            try:
                values = {key: float(row[key]) for key in FIELDS}
                if not all(math.isfinite(v) for v in values.values()):
                    raise ValueError("nonfinite value")
                if values["dt_s"] <= 0 or values["speed_mph"] < 0 or values["compute_us"] < 0 or abs(values["steering"]) > 1.000001:
                    raise ValueError("invalid interval, speed, compute time, or steering")
                values["key"] = (row["connection"], row["segment"])
                values["controller"] = row["controller"]
                rows.append(values)
            except (ValueError, TypeError) as error:
                raise ValueError(f"{path}:{line}: {error}") from error
    if not rows:
        raise ValueError(f"{path}: empty log")
    controllers = {r["controller"] for r in rows}
    if len(controllers) != 1:
        raise ValueError(f"{path}: mixed controllers; use separate run files")
    duration = sum(r["dt_s"] for r in rows)
    rate_energy = 0.0
    segments = 1
    for before, after in zip(rows, rows[1:]):
        if before["key"] == after["key"]:
            rate_energy += (after["steering"]-before["steering"])**2/after["dt_s"]
        else:
            segments += 1
    metrics = {
        "label": label or path.stem, "file": str(path), "controller": rows[0]["controller"],
        "samples": len(rows), "segments": segments, "duration_s": duration,
        "rms_cte_m": math.sqrt(sum(r["cte_m"]**2*r["dt_s"] for r in rows)/duration),
        "max_cte_m": max(abs(r["cte_m"]) for r in rows),
        "rms_steering_rate_per_s": math.sqrt(rate_energy/duration),
        "saturation_percent": 100*sum(r["dt_s"] for r in rows if abs(r["steering"]) >= .999)/duration,
        "mean_speed_mph": sum(r["speed_mph"]*r["dt_s"] for r in rows)/duration,
        "mean_compute_ms": statistics.mean(r["compute_us"] for r in rows)/1000,
        "median_interval_ms": statistics.median(r["dt_s"] for r in rows)*1000,
    }
    return rows, metrics


def make_plots(runs, destination):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    for name in ("timeseries.png", "metrics.png", "summary.csv", "README.txt"):
        if (destination/name).exists():
            raise ValueError(f"Output already exists: {destination/name}; choose a new --out directory")
    colors = plt.get_cmap("tab10")
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True, constrained_layout=True)
    for i, (rows, metrics) in enumerate(runs):
        t, elapsed, previous = [], 0.0, None
        series = {k: [] for k in ("cte_m", "steering", "speed_mph", "compute_us")}
        for row in rows:
            if previous is not None and row["key"] != previous:
                t.append(float("nan"))
                for values in series.values():
                    values.append(float("nan"))
            elapsed += row["dt_s"]
            t.append(elapsed)
            for key, values in series.items():
                values.append(row[key]/1000 if key == "compute_us" else row[key])
            previous = row["key"]
        for ax, values in zip(axes, series.values()):
            ax.plot(t, values, label=metrics["label"], color=colors(i % 10), linewidth=1.15)
    for ax, ylabel in zip(axes, ("CTE (m)", "Steering [-1, 1]", "Actual speed (mph)", "Compute time (ms)")):
        ax.set_ylabel(ylabel)
        ax.grid(alpha=.2)
    axes[1].set_ylim(-1.1, 1.1)
    axes[0].legend(loc="upper left", ncol=min(3,len(runs)), fontsize=8)
    axes[-1].set_xlabel("Accumulated logged time (s); excluded pauses omitted; not track-position aligned")
    fig.suptitle("Simulator controller comparison — full recordings, breaks at reset/pause boundaries")
    fig.savefig(destination/"timeseries.png", dpi=160)
    plt.close(fig)
    specs = [("rms_cte_m", "Tracking error RMS", "m"),
             ("max_cte_m", "Maximum absolute tracking error", "m"),
             ("rms_steering_rate_per_s", "Commanded steering-rate RMS", "normalized units/s"),
             ("saturation_percent", "Time at full steering", "%"),
             ("mean_speed_mph", "Actual mean speed", "mph"),
             ("mean_compute_ms", "Mean controller computation", "ms")]
    fig, axes = plt.subplots(3, 2, figsize=(12, 11), constrained_layout=True)
    labels = [m["label"] for _,m in runs]
    for ax, (key,title,unit) in zip(axes.flat, specs):
        values = [m[key] for _,m in runs]
        bars = ax.bar(range(len(runs)), values, color=[colors(i % 10) for i in range(len(runs))])
        ax.set_xticks(range(len(runs)), labels, rotation=20, ha="right", fontsize=8)
        ax.set_ylim(0,max(max(values)*1.25, .01))
        ax.bar_label(bars, labels=[f"{v:.3g}" for v in values], padding=4)
        ax.set_title(title)
        ax.set_ylabel(unit)
        ax.grid(axis="y",alpha=.2)
        ax.set_axisbelow(True)
    fig.suptitle("Full-log metrics — unequal durations/speeds; lap success is not recorded in CSV")
    fig.savefig(destination/"metrics.png", dpi=160)
    plt.close(fig)
    with (destination/"summary.csv").open("w",newline="",encoding="utf-8") as stream:
        writer=csv.DictWriter(stream,fieldnames=list(runs[0][1]))
        writer.writeheader()
        writer.writerows(metrics for _,metrics in runs)
    (destination/"README.txt").write_text(
        "Each file is one recording, not necessarily one completed lap.\n"
        "All rows contribute; error, speed, and saturation use dt weighting.\n"
        "Steering-rate calculations skip segment boundaries. Compute time is a per-update mean.\n"
        "Time plots concatenate logged intervals and break lines at reset/pause boundaries.\n"
        "These are not distance-aligned laps. Durations and median update intervals are in summary.csv.\n"
        "Target speed and lap success are not in the CSV. Labels are user-provided, not inferred metadata.\n"
        "Compare equal target speeds, starting points, track, and lap counts; repeat runs.\n"
        "A short failed run is not a better controller because its RMS happens to be lower.\n",
        encoding="utf-8")


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs",nargs="+",help="Real simulator CSV files; synthetic benchmark CSVs are different")
    parser.add_argument("--labels",nargs="+",help="One display label per file, in the same order")
    parser.add_argument("--out",default="outputs/run-plots",help="Directory for PNG charts and summary CSV; existing results are not overwritten")
    args=parser.parse_args()
    if args.labels and len(args.labels)!=len(args.logs):
        parser.error("--labels must have one entry per input file")
    labels=args.labels or [None]*len(args.logs)
    try:
        runs=[read_run(path,label) for path,label in zip(args.logs,labels)]
        make_plots(runs,args.out)
    except (ValueError,OSError,ImportError) as error:
        parser.exit(1,f"Error: {error}\n")
    print(f"Saved timeseries.png, metrics.png and summary.csv in {args.out}")


if __name__ == "__main__":
    main()
