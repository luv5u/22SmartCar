#!/usr/bin/env python3
"""Repeat the M3-4 Gazebo experiment. Each invocation owns a new session directory."""
import argparse
import csv
import math
import os
import re
import shutil
import signal
import sqlite3
import subprocess as sp
import struct
import time
import xml.etree.ElementTree as ET
from contextlib import ExitStack, contextmanager
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NAV = ROOT.parent / "M3-3"
PYTHON = "/usr/bin/python3"
STATUS = "/navigate_to_pose/_action/status"
TOPICS = ["/clock", "/odom", "/scan", "/tf", "/tf_static", "/plan",
          "/cmd_vel", STATUS, "/navigate_to_pose/_action/feedback"]
FIELDS = ["run", "weight", "spawn_ok", "outcome", "wall_s", "goal_id",
          "odom_m", "extra_m", "heading_rad_per_m", "box_x_m", "box_y_m", "obstacle_at", "delay_sim_s", "baseline_m", "note"]


@contextmanager
def running(command, log):
    with log.open("w") as output:
        process = sp.Popen(command, stdout=output, stderr=sp.STDOUT,
                           start_new_session=True)
        try:
            yield process
        finally:
            for sig, grace in [(signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 2)]:
                try:
                    os.killpg(process.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    process.wait(timeout=grace)
                    os.killpg(process.pid, 0)
                except ProcessLookupError:
                    break
                except sp.TimeoutExpired:
                    continue
                time.sleep(0.5)


def wait_text(process, log, marker, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"process exited; inspect {log}")
        if marker in log.read_text(errors="replace"):
            return
        time.sleep(.2)
    raise RuntimeError(f"timed out waiting for {marker!r} in {log}")


def ready(launch, folder, planner):
    deadline = time.monotonic() + 120
    while time.monotonic() < deadline:
        if launch.poll() is not None:
            raise RuntimeError("launch exited; inspect launch.log")
        try:
            result = sp.run([PYTHON, str(NAV/"scripts/check_stack.py")],
                            capture_output=True, text=True, timeout=45)
        except sp.TimeoutExpired:
            continue
        (folder/"ready.log").write_text(result.stdout + result.stderr)
        pose = re.search(r"map -> base_footprint: x=([-\d.]+), y=([-\d.]+)", result.stdout)
        if re.search(r"^READY:", result.stdout, re.M) and pose:
            if math.hypot(*map(float, pose.groups())) > .15:
                raise RuntimeError("unexpected start position")
            break
        time.sleep(1)
    else:
        raise RuntimeError("stack not ready; inspect ready.log")
    answer = sp.check_output(["ros2", "param", "get", "/controller_server",
                              "FollowPath.CostCritic.cost_weight"], text=True, timeout=15)
    (folder/"weight.txt").write_text(answer)
    value = float(answer.split()[-1])
    if not math.isclose(value, {"A":3.81,"B":7.62,"C":5.715}[planner]):
        raise RuntimeError(f"unexpected cost weight {value}")
    return value


def bag_metrics(bag, goal, trajectory):
    poses, statuses = [], []
    for path in sorted(bag.glob("*.db3")):
        db = sqlite3.connect(path.resolve().as_uri()+"?mode=ro", uri=True)
        try:
            for t, name, data in db.execute("""
                SELECT m.timestamp,t.name,m.data FROM messages m JOIN topics t ON m.topic_id=t.id
                WHERE t.name IN (?,?) ORDER BY m.timestamp,m.id
            """, ("/odom", STATUS)):
                if data[:2] != b"\x00\x01":
                    raise ValueError("expected little-endian CDR")
                if name == STATUS:
                    count = struct.unpack_from("<I", data, 4)[0]
                    for j in range(count):
                        offset = 8 + j*28
                        if data[offset:offset+16].hex() == goal:
                            statuses.append((t,data[offset+24]))
                else:
                    # CDR alignment is relative to the payload after the 4-byte prefix.
                    pos = 8
                    for _ in range(2):
                        size = struct.unpack_from("<I",data,4+pos)[0]
                        pos = (pos+4+size+3)//4*4
                    xyz_quat = struct.unpack_from("<7d",data,4+(pos+7)//8*8)
                    x,y,_,qx,qy,qz,qw = xyz_quat
                    yaw = math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
                    poses.append((t,x,y,yaw))
        finally:
            db.close()
    begin = min(t for t,s in statuses if s == 2)
    end = min(t for t,s in statuses if t >= begin and s in (4,5,6))
    track = sorted(p for p in poses if begin <= p[0] <= end)
    if len(track) < 2:
        raise ValueError("insufficient odometry")
    distance = sum(math.hypot(b[1]-a[1], b[2]-a[2]) for a,b in zip(track,track[1:]))
    turns = sum(abs(math.remainder(b[3]-a[3],2*math.pi)) for a,b in zip(track,track[1:]))
    with trajectory.open("w", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(["stamp_ns", "odom_x", "odom_y", "yaw_rad"])
        writer.writerows(track)
    return distance, turns/distance if distance > 0 else ""


def trial(folder, planner, args, row):
    config = ROOT/"config"/f"mppi_{planner.lower()}.yaml"
    with ExitStack() as stack:
        launch = stack.enter_context(running(["ros2","launch",str(NAV/"launch/bringup.launch.py"),
                                               f"params_file:={config}"], folder/"launch.log"))
        row["weight"] = ready(launch, folder, planner)
        bag = folder/"bag"
        record = stack.enter_context(running(["ros2","bag","record","--include-hidden-topics",
                                              "-o",str(bag),*TOPICS],folder/"record.log"))
        wait_text(record, folder/"record.log", "All requested topics are subscribed", 45)
        trigger = stack.enter_context(running([PYTHON,"-u",str(ROOT/"scripts/inject_obstacle.py"),
                                               f"--obstacle-at={args.obstacle_at}",
                                               "--delay",str(args.delay)],folder/"trigger.log"))
        wait_text(trigger, folder/"trigger.log","READY",30)
        with running([PYTHON,str(NAV/"scripts/goto.py"),"--x","6","--y","4","--yaw","1.57079633",
                      "--out",str(folder/"goto.csv")],folder/"goal.log") as client:
            client.wait(timeout=180)
        with (folder/"goto.csv").open() as f:
            goal, = list(csv.DictReader(f))
        row.update(outcome=goal["outcome"],wall_s=goal["accept_to_result_wall_s"],goal_id=goal["goal_id"])
        time.sleep(2)
        row["spawn_ok"] = "success=True" in (folder/"trigger.log").read_text()
    distance, smooth = bag_metrics(bag, row["goal_id"], folder/"trajectory.csv")
    row.update(odom_m=distance,heading_rad_per_m=smooth)
    if row["spawn_ok"] and row["outcome"] == "SUCCEEDED":
        row["extra_m"] = distance - args.baseline_m


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--planners", default="A,B")
    parser.add_argument("--runs", type=int, default=10)
    parser.add_argument("--obstacle-at", default="3,-4")
    parser.add_argument("--delay", default=3., type=float)
    parser.add_argument("--out", default=ROOT/"results", type=Path)
    parser.add_argument("--baseline-m", default=8.7051282, type=float)
    args = parser.parse_args()
    planners = [s.strip().upper() for s in args.planners.split(",")]
    if args.runs < 1 or any(p not in ("A","B","C") for p in planners) or len(set(planners)) != len(planners):
        parser.error("choose distinct planners A,B,C and a positive number of runs")
    try:
        x,y = map(float,args.obstacle_at.split(","))
    except ValueError:
        parser.error("--obstacle-at requires X,Y")
    if not all(map(math.isfinite,(x,y,args.delay,args.baseline_m))) or args.delay < 0 or args.baseline_m <= 0:
        parser.error("invalid coordinates, delay or baseline")
    if not shutil.which("ros2"):
        parser.exit(2,"Source the ROS 2 Humble environment first.\n")
    if sp.run(["pgrep","-x","gzserver"],stdout=sp.DEVNULL).returncode == 0:
        parser.exit(2,"Close the existing Gazebo launch before starting this batch.\n")
    session = args.out.resolve()/datetime.now().strftime("batch_%Y%m%d_%H%M%S_%f")
    session.mkdir(parents=True)
    model = ROOT/"models/test_box.sdf"
    shutil.copy2(model, session/"test_box.sdf")
    box = ET.parse(model).findtext(".//collision/geometry/box/size").split()
    print(session,flush=True)
    with (session/"benchmark.csv").open("x",newline="") as f:
        writer=csv.DictWriter(f,fieldnames=FIELDS)
        writer.writeheader()
        for p in planners:
            for i in range(1,args.runs+1):
                label=f"{p}{i:02d}"
                folder=session/label
                folder.mkdir()
                row={"run":label,"outcome":"INCOMPLETE", "box_x_m":box[0], "box_y_m":box[1],
                     "obstacle_at":args.obstacle_at, "delay_sim_s":args.delay, "baseline_m":args.baseline_m}
                try:
                    trial(folder,p,args,row)
                except (Exception, KeyboardInterrupt) as exc:
                    row["note"]=f"{type(exc).__name__}: {exc}"
                    raise
                finally:
                    writer.writerow(row)
                    f.flush()
                    print(row,flush=True)
if __name__=="__main__":
    main()
