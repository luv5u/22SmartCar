"""Summarize comparable Gazebo trials from run_benchmark.py session directories."""
import argparse
import csv
from collections import defaultdict
from pathlib import Path
from statistics import mean

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("batches", type=Path, nargs="+")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    records = defaultdict(list)
    scenes = set()
    seen = set()
    tracks, axis = plt.subplots()
    colors = dict(A="tab:blue", B="tab:orange", C="tab:green")
    for batch in args.batches:
        if batch.resolve() in seen:
            parser.error(f"duplicate batch: {batch}")
        seen.add(batch.resolve())
        with (batch / "benchmark.csv").open() as file:
            for row in csv.DictReader(file):
                scenes.add(tuple(row[key] for key in
                                 ("box_x_m", "box_y_m", "obstacle_at", "delay_sim_s", "baseline_m")))
                group = row["run"][0]
                records[group].append(row)
                track = batch / row["run"] / "trajectory.csv"
                if row["spawn_ok"] == "True" and track.exists():
                    with track.open() as source:
                        points = list(csv.DictReader(source))
                    axis.plot([float(p["odom_x"]) for p in points],
                              [float(p["odom_y"]) for p in points],
                              color=colors[group], alpha=.45)
    if len(scenes) != 1:
        parser.error("expected exactly one box size, position, delay and reference distance")
    args.out.mkdir(parents=True, exist_ok=True)
    metrics = ["success_percent", "wall_s", "extra_m", "heading_rad_per_m"]
    summaries = []
    for group, rows in sorted(records.items()):
        valid = [r for r in rows if r["spawn_ok"] == "True"]
        wins = [r for r in valid if r["outcome"] == "SUCCEEDED"]
        values = {}
        for key in metrics[1:]:
            samples = [float(r[key]) for r in wins if r[key] != ""]
            values[key] = mean(samples) if samples else None
            values[key + "_n"] = len(samples)
        summaries.append(dict(group=group, attempts=len(rows), valid=len(valid), successes=len(wins),
                              success_percent=100*len(wins)/len(valid) if valid else None, **values))
        axis.plot([], [], color=colors[group], label=group)
    with (args.out / "summary.csv").open("w", newline="") as dest:
        writer = csv.DictWriter(dest, fieldnames=summaries[0].keys())
        writer.writeheader()
        writer.writerows(summaries)
    axis.set(xlabel="odom x (m)", ylabel="odom y (m)", aspect="equal")
    axis.legend()
    tracks.savefig(args.out / "trajectories.png", dpi=160, bbox_inches="tight")
    charts, axes = plt.subplots(2, 2, figsize=(8, 6), constrained_layout=True)
    for key, ax in zip(metrics, axes.flat):
        for row in summaries:
            if row[key] is not None:
                ax.bar(row["group"], row[key], color=colors[row["group"]])
        ax.set_title(key)
    charts.savefig(args.out / "metrics.png", dpi=160)


if __name__ == "__main__":
    main()
