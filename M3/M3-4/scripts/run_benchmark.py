#!/usr/bin/env python3
"""Repeat the M3-4 Gazebo experiment. Each invocation owns a new session directory."""
import argparse
import csv
import fcntl
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
    return controller_weight(folder, planner)


def controller_weight(folder, planner):
    import rclpy
    from rcl_interfaces.msg import ParameterType
    from rcl_interfaces.srv import GetParameters

    name = "FollowPath.CostCritic.cost_weight"
    service = "/controller_server/get_parameters"
    expected = {"A": 3.81, "B": 7.62, "C": 5.715}[planner]
    rclpy.init(args=[])
    node = rclpy.create_node("m34_weight_check")
    try:
        client = node.create_client(GetParameters, service)
        if not client.wait_for_service(timeout_sec=20.0):
            raise RuntimeError(f"{service} unavailable after 20 s")
        future = client.call_async(GetParameters.Request(names=[name]))
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        if not future.done():
            raise RuntimeError(f"{service} did not reply within 10 s")
        values = future.result().values
        if len(values) != 1 or values[0].type != ParameterType.PARAMETER_DOUBLE:
            types = [v.type for v in values]
            raise RuntimeError(f"{name}: expected a double; returned types={types} (0=unset)")
        value = values[0].double_value
        if not math.isclose(value, expected):
            raise RuntimeError(f"{name}: expected {expected} for {planner}, got {value}")
        (folder/"weight.txt").write_text(f"{name} = {value}\n")
        return value
    except Exception as error:
        (folder/"weight.txt").write_text(f"{service}: {error}\n")
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


class MissingTrack(ValueError):
    pass


def bag_metrics(bag, goal, trajectory):
    from action_msgs.msg import GoalStatusArray
    from rclpy.serialization import deserialize_message

    if not goal:
        raise MissingTrack("goal client returned no accepted goal UUID")
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
                    for item in deserialize_message(data, GoalStatusArray).status_list:
                        if bytes(item.goal_info.goal_id.uuid).hex() == goal:
                            statuses.append((t, item.status))
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
    begin = min((t for t,s in statuses if s == 2), default=None)
    if begin is None:
        raise MissingTrack(f"no EXECUTING for {goal}; matched states={sorted(set(s for _,s in statuses))}")
    end = min((t for t,s in statuses if t >= begin and s in (4,5,6)), default=None)
    if end is None:
        raise MissingTrack(f"no terminal status for {goal}")
    track = sorted(p for p in poses if begin <= p[0] <= end)
    if len(track) < 2:
        raise MissingTrack("insufficient odometry")
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
    recover_metrics(folder, row)


def recover_metrics(folder, row):
    try:
        distance, smooth = bag_metrics(folder/"bag", row["goal_id"], folder/"trajectory.csv")
    except MissingTrack as error:
        row["note"] = str(error)
        return
    row.update(odom_m=distance, heading_rad_per_m=smooth)
    if str(row["spawn_ok"]) == "True" and row["outcome"] == "SUCCEEDED":
        row["extra_m"] = distance - float(row["baseline_m"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--planners", default="A,B")
    parser.add_argument("--runs", type=int, default=10)
    parser.add_argument("--obstacle-at", default="3,-4")
    parser.add_argument("--delay", default=3., type=float)
    parser.add_argument("--out", default=ROOT/"results", type=Path)
    parser.add_argument("--baseline-m", default=8.7051282, type=float)
    parser.add_argument("--resume", type=Path, help="existing batch directory; complete remaining injected trials")
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
    session = args.resume.resolve() if args.resume else args.out.resolve()/datetime.now().strftime("batch_%Y%m%d_%H%M%S_%f")
    model = ROOT/"models/test_box.sdf"
    if args.resume:
        if not (session/"benchmark.csv").is_file() or (session/"test_box.sdf").read_bytes() != model.read_bytes():
            parser.error("missing original batch CSV or obstacle model differs")
    else:
        session.mkdir(parents=True)
        shutil.copy2(model, session/"test_box.sdf")
    print(session, flush=True)
    box = ET.parse(model).findtext(".//collision/geometry/box/size").split()
    with (session/"batch.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        run_session(args, session, box, planners)


def run_session(args, session, box, planners):
    csv_path = session/"benchmark.csv"
    rows = []
    if args.resume:
        with csv_path.open(newline="") as source:
            reader = csv.DictReader(source)
            if reader.fieldnames != FIELDS:
                raise ValueError("unexpected benchmark CSV format")
            rows = list(reader)
        for row in rows:
            if (row["obstacle_at"] != args.obstacle_at or float(row["delay_sim_s"]) != args.delay
                    or float(row["baseline_m"]) != args.baseline_m):
                raise ValueError("resume with original obstacle, delay and baseline")
        # Retain original exception notes alongside any new diagnosis.
        for row in rows:
            if row["odom_m"] or not row["goal_id"] or not (session/row["run"]/"bag").is_dir():
                continue
            original = row["note"]
            recover_metrics(session/row["run"], row)
            row["note"] = " | ".join(dict.fromkeys(filter(None, (original, row["note"]))))
        temp = session/"benchmark.csv.tmp"
        with temp.open("w", newline="") as dest:
            writer = csv.DictWriter(dest, fieldnames=FIELDS)
            writer.writeheader()
            writer.writerows(rows)
        temp.replace(csv_path)
    with csv_path.open("a" if args.resume else "x", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        if not args.resume:
            writer.writeheader()
        for p in planners:
            valid = sum(row["run"].startswith(p) and row["spawn_ok"] == "True" for row in rows)
            index = max((int(path.name[1:]) for path in session.glob(p+"*")
                         if path.is_dir() and path.name[1:].isdigit()), default=0)
            for _ in range(max(0, args.runs-valid)):
                index += 1
                label = f"{p}{index:02d}"
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
