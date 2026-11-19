"""Evaluate the exported M3-4 analysis ZIP. Run from the M3-4 directory."""
import argparse
import base64
import csv
import io
import math
import statistics
import struct
import zipfile
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument("archive", type=Path)
parser.add_argument("--out", type=Path, default=Path("results"))
args = parser.parse_args()
args.out.mkdir(exist_ok=True, parents=True)

def trajectory(archive, label, goal):
    poses, events = [], []
    with archive.open(label + "/messages.csv") as stream:
        for row in csv.DictReader(io.TextIOWrapper(stream)):
            t = int(row["bag_time_ns"])
            b = base64.b64decode(row["cdr_base64"])
            assert b[:2] == b"\x00\x01", "Expected little-endian CDR"
            if row["topic"] == "/odom":
                # Skip timestamp and both variable-length frame names, aligning in the CDR payload.
                pos = 8
                for _ in range(2):
                    length = struct.unpack_from("<I", b, 4 + pos)[0]
                    pos = (pos + 4 + length + 3) // 4 * 4
                offset = 4 + (pos + 7) // 8 * 8
                x, y, _, qx, qy, qz, qw = struct.unpack_from("<7d", b, offset)
                yaw = math.atan2(2*(qw*qz + qx*qy), 1-2*(qy*qy + qz*qz))
                poses.append((t, x, y, yaw))
            else:
                for j in range(struct.unpack_from("<I", b, 4)[0]):
                    offset = 8 + 28*j
                    uid = b[offset:offset+16].hex()
                    if goal is None or uid == goal:
                        events.append((t, uid, b[offset+24]))
    start, uid, _ = next(e for e in events if e[2] == 2)
    end = next(t for t, u, s in events if u == uid and t >= start and s in (4,5,6))
    path = [(x,y,h) for t,x,y,h in poses if start <= t <= end]
    assert len(path) >= 2
    dist = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a,b in zip(path,path[1:]))
    turn = sum(abs(math.remainder(b[2]-a[2],2*math.pi)) for a,b in zip(path,path[1:]))
    return path, dist, turn/dist

records, paths = [], {}
with zipfile.ZipFile(args.archive) as z:
    paths["baseline"], base, _ = trajectory(z,"baseline",None)
    for name in sorted(p for p in z.namelist() if p.endswith("/goto.csv")):
        label = name.split("/")[0]
        goals = list(csv.DictReader(io.StringIO(z.read(name).decode())))
        first = goals[0]
        valid = "success=True" in z.read(label+"/trigger.log").decode()
        paths[label], length, smooth = trajectory(z,label,first["goal_id"])
        records.append(dict(run=label,valid=int(valid),outcome=first["outcome"],
            wall_s=float(first["accept_to_result_wall_s"]),odom_m=length,
            extra_m=length-base if valid else None,heading_rad_per_m=smooth,
            goal_id=first["goal_id"],extra_requests=len(goals)-1))
with (args.out/"benchmark.csv").open("w",newline="") as f:
    writer=csv.DictWriter(f,fieldnames=records[0])
    writer.writeheader()
    writer.writerows(records)

groups = [[r for r in records if r["run"][0]==g and r["valid"]] for g in "ABC"]
means=[]
for g, rows in zip("ABC",groups):
    ok=[r for r in rows if r["outcome"]=="SUCCEEDED"]
    means.append([100*len(ok)/len(rows)]+[statistics.mean(r[k] for r in ok)
                 for k in ("wall_s","extra_m","heading_rad_per_m")])
    print(g,means[-1])
colors=["#156581","#D47D31","#5A9565"]
fig,axes=plt.subplots(2,2,figsize=(8,6),layout="constrained")
for i,(ax,title) in enumerate(zip(axes.flat,["Success (%)","Navigation time (s)","Extra distance (m)","Heading variation (rad/m)"])):
    bars=ax.bar(list("ABC"),[m[i] for m in means],color=colors)
    ax.set_title(title)
    ax.margins(y=.2)
    ax.bar_label(bars,fmt="%.3f" if i>=2 else "%.2f")
fig.savefig(args.out/"metrics.png",dpi=180)
fig,axes=plt.subplots(1,2,figsize=(10,5),layout="constrained")
for ax in axes:
    for g,rows,col in zip("ABC",groups,colors):
        for j,r in enumerate(rows):
            x,y,_=zip(*paths[r["run"]])
            ax.plot(x,y,color=col,alpha=.65,linewidth=.9,label=g if j==0 else None)
    x,y,_=zip(*paths["baseline"])
    ax.plot(x,y,"k--",linewidth=.9,label="No obstacle")
    ax.add_patch(plt.Rectangle((2.8,-.2),.4,.4,color="gray",alpha=.5,label="Box (nominal)"))
    ax.set_aspect("equal"); ax.set_xlabel("odom x (m)"); ax.set_ylabel("odom y (m)")
axes[0].set_title("All valid runs"); axes[0].legend(fontsize=8)
axes[1].set_title("Obstacle segment"); axes[1].set_xlim(1,4.5); axes[1].set_ylim(-.8,1.4)
fig.savefig(args.out/"trajectories.png",dpi=180)
