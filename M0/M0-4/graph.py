# -*- coding: utf-8 -*-
"""拓扑排序 + 环检测（反复扫描的朴素 Kahn）。"""

import sys


def check_cycle_and_order(tasks):
    """
    tasks: list[dict]，每个有 name / dependencies
    返回按依赖顺序排好的任务列表；有环则报错退出。
    """
    done = set()
    order = []

    while len(order) < len(tasks):
        progressed = False
        for t in tasks:
            if t["name"] in done:
                continue
            if all(d in done for d in t["dependencies"]):
                order.append(t)
                done.add(t["name"])
                progressed = True
                break   # 找到一个就重新扫，保证稳定顺序
        if not progressed:
            left = [t["name"] for t in tasks if t["name"] not in done]
            print(f"错误：检测到依赖环，涉及任务: {', '.join(left)}", file=sys.stderr)
            sys.exit(1)

    return order