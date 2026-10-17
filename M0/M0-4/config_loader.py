# -*- coding: utf-8 -*-
"""配置加载与校验。任何异常都给出可读提示并以非 0 退出。"""

import json
import os
import sys


def _fail(msg):
    print(f"错误：{msg}", file=sys.stderr)
    sys.exit(1)


def load_config(path):
    """读取 yaml/json，返回 dict。"""
    if not os.path.isfile(path):
        _fail(f"配置文件不存在: {path}")

    ext = os.path.splitext(path)[1].lower()

    try:
        if ext in (".yaml", ".yml"):
            import yaml
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
        elif ext == ".json":
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        else:
            _fail(f"不支持的配置文件扩展名: {ext}（应为 .yaml/.yml/.json）")
    except SystemExit:
        raise
    except Exception as e:
        _fail(f"配置文件语法错误: {e}")

    if not isinstance(data, dict):
        _fail("配置文件根节点必须是对象（mapping）")

    return data


def validate_config(data):
    """
    校验配置，返回 (timeout, tasks)。
    timeout 可能为 None。
    """
    timeout = data.get("timeout", None)
    if timeout is not None:
        if not isinstance(timeout, (int, float)) or timeout < 0:
            _fail(f"timeout 必须是非负数字，当前: {timeout!r}")

    tasks = data.get("tasks", None)
    if not tasks or not isinstance(tasks, list):
        _fail("任务列表为空或缺失")

    names = set()

    for i, t in enumerate(tasks):
        if not isinstance(t, dict):
            _fail(f"第 {i+1} 个任务不是对象")

        name = t.get("name")
        if not isinstance(name, str) or not name.strip():
            _fail(f"第 {i+1} 个任务缺少合法的 name")
        if name in names:
            _fail(f"任务名重复: {name!r}")
        names.add(name)

        duration = t.get("duration")
        if not isinstance(duration, (int, float)) or duration < 0:
            _fail(f"任务 {name!r} 的 duration 必须是非负数字，当前: {duration!r}")

        rate = t.get("success_rate")
        if not isinstance(rate, (int, float)) or not (0 <= rate <= 1):
            _fail(f"任务 {name!r} 的 success_rate 超出 [0,1]，当前: {rate!r}")

        deps = t.get("dependencies", [])
        if deps is None:
            deps = []
        if not isinstance(deps, list) or not all(isinstance(d, str) for d in deps):
            _fail(f"任务 {name!r} 的 dependencies 必须是字符串列表")

        t["dependencies"] = deps

    # 依赖存在性检查
    for t in tasks:
        for d in t["dependencies"]:
            if d not in names:
                _fail(f"任务 {t['name']!r} 依赖不存在的任务 {d!r}")
            if d == t["name"]:
                _fail(f"任务 {t['name']!r} 不能依赖自己")

    return timeout, tasks