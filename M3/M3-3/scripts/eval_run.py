"""Evaluate one SQLite ROS 2 Humble recording of one navigation attempt."""
import argparse
import csv
import math
import sqlite3
import struct
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


class Reader:
    """Cursor over a little-endian CDR1 message, aligned after encapsulation."""
    def __init__(self, data):
        if data[:2] != b'\x00\x01':
            raise ValueError('Expected little-endian CDR1')
        self.data = memoryview(data)[4:]
        self.offset = 0

    def take(self, fmt, align):
        self.offset += -self.offset % align
        value = struct.unpack_from('<' + fmt, self.data, self.offset)
        self.offset += struct.calcsize('<' + fmt)
        return value

    def uint(self):
        return self.take('I', 4)[0]

    def string(self):
        size = self.uint()
        return self.take(f'{size}s', 1)[0][:-1].decode()

    def header(self):
        seconds, ns = self.take('iI', 4)
        return seconds + ns * 1e-9, self.string()

    def pose(self):
        x, y, _, a, b, c, d = self.take('7d', 8)
        return x, y, math.atan2(2*(d*c+a*b), 1-2*(b*b+c*c))


def evaluate(db, out):
    def rows(topic):
        return db.execute('SELECT timestamp,data FROM messages JOIN topics ON '
                          'topics.id=topic_id WHERE name=? ORDER BY timestamp,messages.id', (topic,))

    states = {}
    for stamp, data in rows('/navigate_to_pose/_action/status'):
        r = Reader(data)
        for _ in range(r.uint()):
            goal = r.take('16s', 1)[0]
            r.take('iI', 4)
            status = r.take('b', 1)[0]
            states.setdefault(goal, []).append((stamp, status))
    if len(states) > 1:
        raise ValueError('Expected one goal per recording')
    events = next(iter(states.values()), [])
    start = next((t for t,s in events if s == 2), None)
    end = next((t for t,s in events if s in (4,5,6)), None)
    status = {1:'ACCEPTED',2:'EXECUTING',3:'CANCELING',4:'SUCCEEDED',5:'CANCELED',
              6:'ABORTED'}.get(events[-1][1], 'UNKNOWN') if events else 'NO_GOAL_STATUS'
    def window(topic):
        return ((t, Reader(b)) for t,b in rows(topic)
                if start is not None and t >= start and (end is None or t <= end))

    plan = []
    for _, r in window('/plan'):
        _, frame = r.header()
        if frame != 'map':
            raise ValueError('Expected plan in map')
        for _ in range(r.uint()):
            r.header()
            plan.append(r.pose()[:2])
        if plan:
            break
    length = np.linalg.norm(np.diff(plan, axis=0),axis=1).sum() if plan else None
    speed = [(t, *r.take('6d',8)) for t,r in window('/cmd_vel')]
    tf = []
    for _, data in rows('/tf'):
        r = Reader(data)
        for _ in range(r.uint()):
            stamp, parent = r.header()
            child = r.string()
            pose = r.pose()
            if (parent, child) == ('map', 'odom'):
                tf.append((stamp, *pose))
    odom = []
    for _, r in window('/odom'):
        stamp, parent = r.header()
        r.string()
        if parent == 'odom':
            odom.append((stamp,*r.pose()))
    fig, axes = plt.subplots(1,3,figsize=(13,4), constrained_layout=True)
    if plan:
        axes[0].plot(*np.asarray(plan).T, '--', label='First plan')
    if tf and odom:
        ts,x,y,_ = np.asarray(odom).T
        tt,tx,ty,yaw = np.asarray(sorted(tf)).T
        dx,dy,a = [np.interp(ts,tt,v) for v in (tx,ty,np.unwrap(yaw))]
        axes[0].plot(dx+x*np.cos(a)-y*np.sin(a),dy+x*np.sin(a)+y*np.cos(a),label='TF + odom')
    if axes[0].lines:
        axes[0].legend()
    axes[0].set(aspect='equal',xlabel='map x (m)',ylabel='map y (m)')
    if speed:
        t,v,_,_,_,_,w = np.asarray(speed).T
        axes[1].plot((t-start)/1e9,v,label='vx (m/s)')
        axes[1].plot((t-start)/1e9,w,label='wz (rad/s)')
        axes[1].legend()
    axes[1].set(xlabel='Receipt time since EXECUTING (s)',title='Final /cmd_vel')
    def costmap(axis, topic):
        costs = list(window(topic))
        if costs:
            _,r = costs[len(costs)//2]
            _,frame = r.header()
            r.take('iI',4)
            res, = r.take('f',4)
            width,height = r.take('II',4)
            x,y,_ = r.pose()
            count = r.uint()
            grid = np.frombuffer(r.data,dtype='i1',count=count,offset=r.offset).reshape(height,width)
            axis.imshow(np.ma.masked_less(grid,0),origin='lower',vmin=0,vmax=100,
                           extent=(x,x+width*res,y,y+height*res),cmap='magma',interpolation='nearest')
            axis.set(xlabel=f'{frame} x (m)',ylabel=f'{frame} y (m)')
            return True
        return False

    costmap(axes[2], '/local_costmap/costmap')
    axes[2].set_title('Local costmap at middle sample')
    # Keep the original composite and export standalone evidence for the demo.
    for label in ('global_costmap', 'local_costmap', 'cmd_vel'):
        evidence, axis = plt.subplots(figsize=(6, 4), constrained_layout=True)
        if label == 'cmd_vel':
            available = bool(speed)
            if available:
                for line in axes[1].lines:
                    axis.plot(line.get_xdata(), line.get_ydata(), label=line.get_label())
                axis.set_xlabel('Receipt time since EXECUTING (s)')
                axis.legend()
        else:
            available = costmap(axis, '/' + label + '/costmap')
        if available:
            axis.set_title(label)
            evidence.savefig(out.parent / f'{out.name}_{label}.png', dpi=160)
        plt.close(evidence)
    fig.suptitle(status)
    fig.savefig(out.with_suffix('.png'),dpi=160)
    plt.close(fig)
    result = dict(bag=out.name,status=status,first_plan_m=length,
                  execution_s=(end-start)/1e9 if start is not None and end is not None else None,
                  peak_cmd_vx=max((row[1] for row in speed), default=None))
    with out.with_suffix('.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f,fieldnames=result)
        writer.writeheader()
        writer.writerow(result)
    print(result)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag',type=Path,help='Single .db3 file or single-segment bag directory')
    parser.add_argument('--out',type=Path,default=Path('results'))
    args = parser.parse_args()
    files = sorted(args.bag.glob('*.db3')) if args.bag.is_dir() else [args.bag]
    if len(files) != 1 or not files[0].is_file():
        parser.error('Expected exactly one SQLite bag file')
    args.out.mkdir(parents=True,exist_ok=True)
    with sqlite3.connect(files[0].resolve().as_uri()+'?mode=ro',uri=True) as db:
        evaluate(db,args.out/files[0].stem.removesuffix('_0'))
