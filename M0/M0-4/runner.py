# -*- coding: utf-8 -*-
"""执行调度：依赖检查、重试、超时中断。"""

import random
import time

from logger import log

SUCCESS = "SUCCESS"
FAILED  = "FAILED"
SKIPPED = "SKIPPED"
TIMEOUT = "TIMEOUT"
PENDING = "PENDING"

MAX_ATTEMPTS = 3


def run(tasks, order, timeout, seed):
    """
    返回 (result_dict, timed_out, total_duration)
    result_dict: name -> 结果字典（按任务名索引）
    """
    rng = random.Random(seed)
    begin = time.time()

    # 初始化结果
    result = {}
    for t in tasks:
        result[t["name"]] = {
            "name": t["name"],
            "status": PENDING,
            "attempts": 0,
            "duration": 0.0,
            "started_at": None,
            "ended_at": None,
        }

    timed_out = False

    for t in order:
        name = t["name"]
        r = result[name]

        # (1) 执行前检查超时
        if timeout is not None and time.time() - begin > timeout:
            timed_out = True
            log(TIMEOUT, name, f"任务开始前已超时（{time.time()-begin:.2f}s > {timeout}s）")
            break

        # (2) 检查依赖：有一个不是 SUCCESS 就跳过
        dep_ok = True
        for d in t["dependencies"]:
            if result[d]["status"] != SUCCESS:
                dep_ok = False
                break
        if not dep_ok:
            r["status"] = SKIPPED
            log(SKIPPED, name, "前置任务未成功")
            continue

        # (3) 执行 + 最多重试 2 次（共 3 次尝试）
        r["started_at"] = time.time()
        success = False

        for attempt in range(1, MAX_ATTEMPTS + 1):
            # 重试前也检查超时
            if timeout is not None and time.time() - begin > timeout:
                timed_out = True
                log(TIMEOUT, name, f"重试前已超时（{time.time()-begin:.2f}s > {timeout}s）")
                break

            r["attempts"] = attempt
            if attempt > 1:
                log("RETRY", name, f"第 {attempt}/{MAX_ATTEMPTS} 次尝试")

            time.sleep(t["duration"])
            r["duration"] += t["duration"]

            # sleep 后立即检查超时（关键：防止最后一个任务睡过头）
            if timeout is not None and time.time() - begin > timeout:
                timed_out = True
                log(TIMEOUT, name, f"执行中超时（{time.time()-begin:.2f}s > {timeout}s）")
                break

            if rng.random() < t["success_rate"]:
                success = True
                break
            else:
                log(FAILED, name, f"第 {attempt} 次尝试失败")

        r["ended_at"] = time.time()

        if timed_out:
            r["status"] = TIMEOUT
            break

        if success:
            r["status"] = SUCCESS
            log(SUCCESS, name, f"{r['duration']:.2f}s")
        else:
            r["status"] = FAILED
            log(FAILED, name, f"{MAX_ATTEMPTS} 次尝试均失败")

    # (4) 没执行到的任务全部标 TIMEOUT
    if timed_out:
        for name, r in result.items():
            if r["status"] == PENDING:
                r["status"] = TIMEOUT

    total = time.time() - begin
    return result, timed_out, total