#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sensor_analyzer.py  —— 上一届学长留下的"能用"的脚本

注释（学长原话）：
    "处理一下传感器数据就能用"

原本意图：
    1. 读取 sensor_data.csv（列：time, value）
    2. 计算 value 的平均值、标准差
    3. 剔除离群值（|value - mean| > 2 * std）
    4. 把清洗后的数据保存为 cleaned_data.csv
    5. 打印一份统计摘要

现状：跑不通 / 跑出来数不对。就交给你了。
"""

import csv
import os
import math
import argparse
import sys

def fail(msg):
    print("错误：%s" % msg, file=sys.stderr)
    sys.exit(1)

#单独封装读取相关函数并进行异常处理
def load_data(path):
    if not os.path.isfile(path):
        fail("输入文件不存在：%s" % path)

    rows = []
    values = []
    try:
        with open(path, "r", newline="", encoding="utf-8-sig") as f:
            reader = csv.DictReader(f)
            if reader.fieldnames is None:
                fail("输入文件为空或没有表头：%s" % path)

            fieldnames = [name.strip() for name in reader.fieldnames]
            if "time" not in fieldnames or "value" not in fieldnames:
                fail("列名不符，需要包含 'time' 和 'value' 两列，实际表头：%s"
                     % reader.fieldnames)

            for lineno, row in enumerate(reader, start=2):  # 第 2 行起为数据
                # 跳过完全空行
                if row is None or all(
                    (v is None or str(v).strip() == "") for v in row.values()
                ):
                    continue
                try:
                    # 校验 time（不强求使用，但必须是数值）
                    float(row["time"])
                    v = float(row["value"])
                except (ValueError, TypeError):
                    fail("第 %d 行包含非数值内容：%r" % (lineno, row))
                except KeyError as e:
                    fail("第 %d 行缺少列 %s" % (lineno, e))
                rows.append(row)
                values.append(v)
    except UnicodeDecodeError:
        fail("文件编码无法按 UTF-8 读取：%s" % path)
    except csv.Error as e:
        fail("CSV 解析失败：%s" % e)
    except OSError as e:
        fail("无法读取文件 %s：%s" % (path, e))

    return rows, values

#单独封装平均值与标准差计算
def compute_stats(values):
    n = len(values)
    mean = sum(values) / n
    var = sum((v - mean) ** 2 for v in values) / n  # 总体标准差
    return mean, math.sqrt(var)

def main():

    parser = argparse.ArgumentParser(description="传感器数据分析")
    parser.add_argument("--input", help="输入CSV文件路径")
    parser.add_argument("--output", help="输出CSV文件路径")
    args = parser.parse_args()

    CURRENT_DIR = os.path.dirname(os.path.abspath(__file__)) #获取当前绝对路径
    INPUT_FILE = args.input or os.path.join(CURRENT_DIR,"sensor_data.csv")
    OUTPUT_FILE = args.output or "cleaned_data.csv"

    data = []
    cleaned = []

    print("=== 传感器数据分析 ===")

    rows, values = load_data(INPUT_FILE) #读取数据

    if len(values) == 0:
        fail("输入文件没有任何有效数据行：%s" % INPUT_FILE)

    print("共读取 %d 条数据" % len(values))

    mean, std = compute_stats(values) #计算平均值与标准差

    # --- 剔除离群值 ---
    with open(INPUT_FILE,"r",newline='',encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            v = float(row["value"])
            if abs(v - mean) <= 2 * std:
                cleaned.append(row) #整行放入

    # --- 输出清洗后的数据 ---
    output_path = os.path.join(OUTPUT_DIR, OUTPUT_FILE)
    with open(output_path,"w",newline='',encoding='utf-8') as f:
        writer = csv.DictWriter(f,fieldnames=["time", "value"])
        writer.writeheader() #写表头
        writer.writerows(cleaned) #写出所有正常行

    print("均值 mean = %.4f" % mean)
    print("标准差 std = %.4f" % std)
    print("清洗后剩余 %d 条" % len(cleaned))
    print("已保存到 %s" % output_path)

if __name__ == "__main__":
    main()