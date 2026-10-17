#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TaskScheduler — 命令行任务调度模拟器（主入口）"""

import argparse
import json
import sys

from config_loader import load_config, validate_config
from graph import check_cycle_and_order
from runner import run


def build_parser():
    p = argparse.ArgumentParser(
        description="TaskScheduler — 命令行任务调度模拟器"
    )
    p.add_argument("--config", required=True,
                   help="任务配置文件（.yaml/.yml/.json）")
    p.add_argument("--timeout", type=float, default=None,
                   help="覆盖配置文件中的全局超时（秒）")
    p.add_argument("--report", default="report.json",
                   help="报告输出路径（默认 report.json）")
    p.add_argument("--seed", type=int, default=None,
                   help="随机种子，便于复现")
    return p


def main():
    args = build_parser().parse_args()

    # 1) 加载 + 校验
    data = load_config(args.config)
    cfg_timeout, tasks = validate_config(data)

    # timeout 优先级：命令行 > 配置文件
    timeout = args.timeout if args.timeout is not None else cfg_timeout

    # 2) 拓扑排序（含环检测）
    order = check_cycle_and_order(tasks)

    # 3) 执行
    print(f"开始调度：共 {len(tasks)} 个任务，timeout={timeout}s，seed={args.seed}",
          flush=True)
    result, timed_out, total = run(tasks, order, timeout, args.seed)

    # 4) 生成报告（按配置顺序）
    report = {
        "timeout": bool(timed_out),
        "total_duration": round(total, 2),
        "tasks": [result[t["name"]] for t in tasks],
    }

    try:
        with open(args.report, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
    except Exception as e:
        print(f"错误：写报告失败: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"\n报告已写入: {args.report}", flush=True)
    print(f"总耗时: {total:.2f}s，超时: {timed_out}", flush=True)


if __name__ == "__main__":
    main()