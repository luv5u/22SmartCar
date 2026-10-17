# -*- coding: utf-8 -*-
"""彩色日志：非 TTY 环境自动降级为无色输出。"""

import sys

USE_COLOR = sys.stdout.isatty()

COLORS = {
    "SUCCESS": "\033[92m",   # 绿
    "FAILED":  "\033[91m",   # 红
    "RETRY":   "\033[93m",   # 黄
    "SKIPPED": "\033[94m",   # 蓝
    "TIMEOUT": "\033[95m",   # 品红
    "INFO":    "\033[96m",   # 青
}

RESET = "\033[0m"


def log(status, name, msg=""):
    """status: SUCCESS/FAILED/RETRY/SKIPPED/TIMEOUT/INFO"""
    color = COLORS.get(status, "") if USE_COLOR else ""
    reset = RESET if USE_COLOR else ""
    text = f"[{status}] {color}{name}{reset}"
    if msg:
        text += f" — {msg}"
    print(text, flush=True)