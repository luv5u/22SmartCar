#!/usr/bin/env python3
"""M3-1 离线分析：原始 CSV 校验、闭环统计、等比例轨迹图。

无需 ROS/Gazebo；仅绘图依赖 matplotlib。
原始闭环统计严格使用 CSV 起终点，不使用给定 summary.csv 代替计算。
默认图把模型真值沿车体 x 轴平移 -0.14 m，换到后轴中心；这是基于
当前提供模型与阿克曼运动学的参考点解释，必须在 README 中说明。
同时输出未转换参考点的图，保留比较依据。所有操作均为离线分析。
"""

import argparse
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
import statistics
import sys


@dataclass(frozen=True)
class Point:
    t: float
    x: float
    y: float
    theta: float


def normalize(angle):
    """返回 (-pi, pi] 内的角度，避免跨越正负 pi 时误算。"""
    result = (angle + math.pi) % (2 * math.pi) - math.pi
    return math.pi if result <= -math.pi else result


def read_trajectory(path, gap_warning):
    points = []
    with path.open(newline='', encoding='utf-8-sig') as handle:
        reader = csv.reader(handle)
        if next(reader, None) != ['t', 'x', 'y', 'theta']:
            raise ValueError(f'{path}: 表头必须严格为 t,x,y,theta')
        for line, row in enumerate(reader, start=2):
            if len(row) != 4:
                raise ValueError(f'{path}:{line}: 必须恰好四列')
            try:
                values = list(map(float, row))
            except ValueError as error:
                raise ValueError(f'{path}:{line}: 存在非数值字段') from error
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f'{path}:{line}: 存在 NaN 或 Inf')
            point = Point(*values)
            if not -math.pi - 1e-8 <= point.theta <= math.pi + 1e-8:
                raise ValueError(f'{path}:{line}: theta 超出弧度范围')
            if points and point.t < points[-1].t - 1e-9:
                raise ValueError(f'{path}:{line}: 时间倒退，可能混入复位数据')
            points.append(point)
    if len(points) < 3 or points[-1].t <= points[0].t:
        raise ValueError(f'{path}: 有效样本或时间跨度不足')
    intervals = [b.t - a.t for a, b in zip(points, points[1:])]
    checks = {
        'file': str(path), 'samples': len(points),
        'duration_s': points[-1].t - points[0].t,
        'repeated_timestamps': sum(abs(dt) < 1e-9 for dt in intervals),
        'largest_timestamp_gap_s': max(intervals),
        'gap_warning': max(intervals) > gap_warning,
    }
    return points, checks


def closure(points):
    start, end = points[0], points[-1]
    distance = math.hypot(end.x - start.x, end.y - start.y)
    angle = abs(math.degrees(normalize(end.theta - start.theta)))
    return distance, angle


def cumulative_angle(points):
    return math.degrees(sum(normalize(b.theta - a.theta)
                            for a, b in zip(points, points[1:])))


def local_points(points, offset_x=0.0):
    """固定参考点转换，然后仅按初始位姿对齐，不拟合整条轨迹。"""
    shifted = [Point(p.t,
                     p.x + offset_x * math.cos(p.theta),
                     p.y + offset_x * math.sin(p.theta), p.theta)
               for p in points]
    first = shifted[0]
    c, s = math.cos(first.theta), math.sin(first.theta)
    return [Point(p.t, c * (p.x-first.x) + s * (p.y-first.y),
                  -s * (p.x-first.x) + c * (p.y-first.y),
                  normalize(p.theta-first.theta)) for p in shifted]


def discover_pairs(directory, expected):
    pairs = {}
    for path in sorted(directory.glob('run_*.csv')):
        match = re.fullmatch(r'run_(\d+)_(odom|truth)\.csv', path.name)
        if match:
            run, source = int(match[1]), match[2]
            if run in pairs and source in pairs[run]:
                raise ValueError(f'第 {run} 次的 {source} 文件重复')
            pairs.setdefault(run, {})[source] = path
    if sorted(pairs) != list(range(1, expected + 1)):
        raise ValueError(f'需有连续 1–{expected} 次数据，实际编号为 {sorted(pairs)}')
    for run, files in pairs.items():
        if set(files) != {'odom', 'truth'}:
            raise ValueError(f'第 {run} 次缺少 odom 或 truth 文件')
    return pairs


def ideal_path(side, radius):
    x = y = angle = 0.0
    xs, ys = [x], [y]
    for _ in range(4):
        length = side - 2 * radius
        x += length * math.cos(angle)
        y += length * math.sin(angle)
        xs.append(x)
        ys.append(y)
        old_x, old_y, old_angle = x, y, angle
        for step in range(1, 61):
            delta = (math.pi / 2) * step / 60
            x = old_x + radius * (math.sin(old_angle+delta)-math.sin(old_angle))
            y = old_y + radius * (-math.cos(old_angle+delta)+math.cos(old_angle))
            xs.append(x)
            ys.append(y)
        angle += math.pi / 2
    return xs, ys


def save_plot(odom, truth, args, filename, raw=False):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    offset = 0.0 if raw else args.truth_offset_x
    traces = {'odom': local_points(odom), 'truth': local_points(truth, offset)}
    fig, ax = plt.subplots(figsize=(8, 7.5), constrained_layout=True)
    colors = {'odom': '#cf3f3a', 'truth': '#17875b'}
    for source, points in traces.items():
        ax.plot([p.x for p in points], [p.y for p in points],
                color=colors[source], linewidth=2, label=source)
        ax.scatter(points[-1].x, points[-1].y, color=colors[source],
                   marker='x', s=85, zorder=5, label=f'{source} end')
    if not raw:
        xs, ys = ideal_path(args.side, args.radius)
        ax.plot(xs, ys, '--', color='#8a8a8a', linewidth=1.2,
                label='ideal rear-axle path')
    ax.scatter(0, 0, s=55, facecolor='white', edgecolor='#111827',
               zorder=6, label='start (initial pose aligned)')
    dimension_y = args.side + 0.28
    ax.annotate('', xy=(-args.radius, dimension_y),
                xytext=(args.side-args.radius, dimension_y),
                arrowprops={'arrowstyle': '|-|', 'color': '#4b5563'})
    ax.text(args.side/2-args.radius, dimension_y+0.04,
            f'target side = {args.side:g} m', ha='center', va='bottom')
    od, oa = closure(odom)
    td, ta = closure(truth)
    ax.set_title(f'Run {args.plot_run:02d}: trajectory comparison\n'
                 f'Raw closure: odom {od:.4f} m / {oa:.2f} deg; '
                 f'truth {td:.4f} m / {ta:.2f} deg', fontsize=11)
    ax.text(.02, .98,
            f'Truth body-x offset: {offset:+.3f} m\n'
            + ('Raw references differ; not a drift-only comparison.' if raw
               else 'Assumed common reference: rear-axle center.\n'
                    'Fixed geometry transform; no trajectory fitting.'),
            transform=ax.transAxes, va='top', fontsize=9,
            bbox={'facecolor':'white', 'alpha':.85, 'edgecolor':'#dddddd'})
    ax.set_xlabel('x in initial-heading frame (m)')
    ax.set_ylabel('y in initial-heading frame (m)')
    ax.set_aspect('equal', adjustable='box')
    ax.grid(alpha=.2)
    ax.margins(x=.12)
    all_y = [p.y for points in traces.values() for p in points]
    ax.set_ylim(min(min(all_y)-.18, -.18),
                max(max(all_y)+.7, dimension_y+.55))
    ax.legend(loc='upper center', bbox_to_anchor=(.5, -.12),
              ncol=3, fontsize=8)
    fig.savefig(filename, dpi=180)
    plt.close(fig)


def write_report(rows, checks, args, out):
    fields = list(rows[0])
    with (out/'closed_loop_stats.csv').open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    metrics = fields[1:]
    means = {key: statistics.mean(row[key] for row in rows) for key in metrics}
    sds = {key: statistics.stdev(row[key] for row in rows)
           if len(rows) > 1 else None for key in metrics}
    lines = ['# M3-1 闭环误差统计', '',
             f'数据目录：`{args.logs}`；实际次数：{len(rows)}。', '',
             '## 原始 CSV 起终点偏差', '',
             '位置偏差为起终点距离；角度偏差先归一化到 (-π, π]，再取绝对值。',
             '标准差使用样本标准差（分母 n−1）。这里不做参考点转换，便于复核原始记录。', '',
             '| 次数 | odom 位置 (m) | odom 角度 (°) | truth 位置 (m) | truth 角度 (°) |',
             '|---:|---:|---:|---:|---:|']
    primary = ['odom_position_m', 'odom_angle_deg', 'truth_position_m', 'truth_angle_deg']
    for row in rows:
        lines.append('| '+str(row['run'])+' | '+' | '.join(f'{row[k]:.6f}' for k in primary)+' |')
    def stat(key):
        sd = sds[key]
        return f'{means[key]:.6f} ± {sd:.6f}' if sd is not None else f'{means[key]:.6f}（仅一次）'
    lines += ['| 均值 ± 标准差 | '+' | '.join(stat(k) for k in primary)+' |', '',
              '## 参考点与图示', '',
              f'- 图选取固定编号第 {args.plot_run} 次，不根据误差挑选“最好”的一次。',
              '- 红线为 odom，绿线为 Gazebo 真值；两图均使用等比例坐标。',
              f'- 正式比较图对真值施加车体 x 方向 {args.truth_offset_x:+.3f} m 的固定偏移。',
              '- 当前模型后轴在车体中心后方 0.14 m；把控制器运动学参考点解释为后轴中心。',
              '- 这是一项需结合模型/TF 说明的参考点假设，不是利用真值修正里程计。',
              '- 两来源分别只用本圈初始位姿平移、旋转，不做逐点拟合、缩放或终点对齐。',
              '- `trajectory_compare_raw.png` 保留未转换参考点的对照；不能把其中全部差距当作漂移。',
              '- 图中标注的闭环偏差与上表相同，来自原始参考点，不是转换后轨迹的端点距离。', '',
              '## 误差解释的范围', '',
              '- 真值闭环偏差描述实际运动没有完全回到起点的程度。',
              '- odom 闭环偏差描述里程计推算的起终点差；二者不能直接相减当作定位误差。',
              '- 在比较均值与标准差之外，还需结合带符号偏差、轨迹与关节状态判断误差来源。',
              '- 本程序不根据数据大小自动宣称考核通过，也不只凭一个统计量断言滑动或标定故障。', '',
              '## 数据检查', '',
              '| 文件 | 行数 | 时长 (s) | 重复时间戳数 | 最大时间间隔 (s) |',
              '|---|---:|---:|---:|---:|']
    for check in checks:
        lines.append(f'| {Path(check["file"]).name} | {check["samples"]} | '
                     f'{check["duration_s"]:.3f} | {check["repeated_timestamps"]} | '
                     f'{check["largest_timestamp_gap_s"]:.3f} |')
    lines += ['', '重复时间戳不会被擅自删除：/clock 更新慢于轨迹话题时可能出现此现象。',
              '程序允许非递减时间；拒绝倒退、缺列、非数值和非有限数值。',
              '时间戳只能检查已记录时间的一致性，不能仅凭四列 CSV 证明时间来自 /clock。', '']
    (out/'closed_loop_stats.md').write_text('\n'.join(lines), encoding='utf-8')
    metadata = {'source_directory': str(Path(args.logs).resolve()),
                'expected_runs': args.expected_runs, 'plot_run': args.plot_run,
                'side_m': args.side, 'radius_m': args.radius,
                'truth_body_x_offset_m': args.truth_offset_x,
                'sample_standard_deviation': True, 'means': means,
                'standard_deviations': sds, 'checks': checks,
                'warnings': [c['file']+': 时间间隔过大' for c in checks if c['gap_warning']]}
    (out/'analysis_checks.json').write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
    for key in primary:
        print(f'{key}: {stat(key)}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--logs', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--expected-runs', type=int, default=10)
    parser.add_argument('--plot-run', type=int, default=1)
    parser.add_argument('--side', type=float, default=2.0)
    parser.add_argument('--radius', type=float, default=0.6)
    parser.add_argument('--truth-offset-x', type=float, default=-0.14)
    parser.add_argument('--gap-warning', type=float, default=0.5)
    parser.add_argument('--overwrite', action='store_true')
    args = parser.parse_args()
    try:
        if not (args.expected_runs >= 1 and 1 <= args.plot_run <= args.expected_runs
                and all(math.isfinite(v) for v in (args.side, args.radius,
                        args.truth_offset_x, args.gap_warning))
                and 0 < 2*args.radius < args.side and args.gap_warning > 0):
            raise ValueError('次数、画图编号、边长、半径或时间间隔阈值不合法')
        pairs = discover_pairs(Path(args.logs), args.expected_runs)
        trajectories, rows, checks = {}, [], []
        for run in sorted(pairs):
            data = {}
            for source in ('odom', 'truth'):
                data[source], check = read_trajectory(pairs[run][source], args.gap_warning)
                checks.append(check)
            if (abs(data['odom'][0].t-data['truth'][0].t) > args.gap_warning
                    or abs(data['odom'][-1].t-data['truth'][-1].t) > args.gap_warning):
                raise ValueError(f'第 {run} 次两来源的起终点时间不一致')
            row = {'run': run}
            for source in ('odom', 'truth'):
                row[source+'_position_m'], row[source+'_angle_deg'] = closure(data[source])
                row[source+'_total_yaw_deg'] = cumulative_angle(data[source])
            rows.append(row)
            trajectories[run] = data
        out = Path(args.out)
        names = ['closed_loop_stats.csv', 'closed_loop_stats.md',
                 'analysis_checks.json', 'trajectory_compare.png', 'trajectory_compare_raw.png']
        if not args.overwrite and any((out/name).exists() for name in names):
            raise ValueError('已有同名分析结果；请换输出目录或明确加 --overwrite')
        try:
            import matplotlib  # noqa: F401
        except ImportError as error:
            raise ValueError('当前 Python 环境没有 matplotlib，请先在该环境安装绘图库') from error
        out.mkdir(parents=True, exist_ok=True)
        chosen = trajectories[args.plot_run]
        save_plot(chosen['odom'], chosen['truth'], args, out/'trajectory_compare.png')
        save_plot(chosen['odom'], chosen['truth'], args, out/'trajectory_compare_raw.png', raw=True)
        write_report(rows, checks, args, out)
        gaps = sum(c['gap_warning'] for c in checks)
        print(f'已检查 {len(rows)} 次、{len(checks)} 份轨迹；结果写入 {out.resolve()}')
        if gaps:
            print(f'注意：{gaps} 份文件有较大的时间间隔，请看 analysis_checks.json')
        return 0
    except (ValueError, OSError) as error:
        print(f'分析失败：{error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
