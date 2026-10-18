import yaml
import csv
import math
import os
import argparse
import sys
import numpy as np

#自定义异常
class CorrToolError(Exception):
    pass

#封装yaml读取与配置读取
def load_cfg (CONFIG_PATH = "config.yaml"): #存储yaml文件路径
    if not os.path.exists(CONFIG_PATH):
        raise CorrToolError(f"配置文件不存在：{CONFIG_PATH}")
    
    with open(CONFIG_PATH) as f:
        cfg = yaml.safe_load(f)

    if cfg is None:
        raise CorrToolError(f"配置文件为空：{CONFIG_PATH}")

    for k in ("input_csv","colums"):
        if k not in cfg:
            raise CorrToolError(f"配置文件缺少字段：{k}")
    for k in ("x","y"):
        if k not in cfg["columns"]:
            raise CorrToolError(f"配置文件的columns缺少字段：{k}")
    
    #将其中配置提取出
    csv_path = cfg["input_csv"]
    col_x = cfg["columns"]["x"]
    col_y = cfg["columns"]["y"]

    return csv_path, col_x, col_y

#封装csv内数据读取
def load_csv (csv_path, col_x, col_y):
    if not os.path.exists(csv_path):
        raise CorrToolError(f"CSV文件不存在：{csv_path}")
    
    xs = []
    ys = []

    with open(csv_path) as f:
        reader = csv.DictReader(f)
        
        if reader.fieldnames is None:
            raise CorrToolError(f"CSV文件为空：{csv_path}")
        if col_x not in reader.fieldnames:
            raise CorrToolError(f"CSV列名不匹配：{col_x}")
        if col_y not in reader.fieldnames:
            raise CorrToolError(f"CSV列名不匹配：{col_y}")
        
        #将读取的数据转化为列表
        for row in reader:
            xs.append(float(row[col_x]))
            ys.append(float(row[col_y]))
    if len(xs) == 0:
        raise CorrToolError(f"CSV文件没有数据行：{csv_path}")
    if len(xs) < 2:
        raise CorrToolError("有效数据小于2行")
    
    return xs, ys

#封装一般方式计算相关系数
def corr_math (xs, ys):
    n = len(xs)

    if len(xs) != len(ys):
        raise CorrToolError("两列长度不一致")
    
    #得到x，y各自的总和
    sum_x = 0.0
    sum_y = 0.0
    for i in range(n):
        sum_x = sum_x + xs[i]
        sum_y = sum_y + ys[i]

    #得到x，y的平均值
    mean_x = sum_x / n
    mean_y = sum_y / n

    dx = 0.0
    dy = 0.0
    prod = 0.0
    for i in range(n):
        a = xs[i] - mean_x
        b = ys[i] - mean_y

        #得到x，y各自与平均值差的平方的和
        dx = dx + a * a
        dy = dy + b * b

        prod = prod + a * b #x，y与各自平均值差的乘积的和

    if dx == 0.0 or dy == 0.0:
        raise CorrToolError("某一列取值恒定，方差为0,相关系数无定义")
    
    denom = math.sqrt(dx * dy) #开方
    r = prod / denom #得到相关系数
    return n, mean_x, mean_y, r

#使用numpy验算结果并封装
def corr_numpy(xs,ys):
    #x，y分别变为数组
    xn = np.array(xs)
    yn = np.array(ys)

    r_np = np.corrcoef(xn,yn)[0,1] #计算相关系数矩阵并变换为相关系数
    return r_np

#支持--config指定配置文件也支持位置参数
def parse_args():
    parse = argparse.ArgumentParser(description="计算两列数据的相关系数")
    parse.add_argument("--config",dest="config", default=None,help="YAML配置文件路径")
    parse.add_argument("config_pos",nargs="?", default=None,help="YAML配置文件路径（位置参数写法）")
    return parse.parse_args()

if __name__ == '__main__': #保护
    try:
        args = parse_args()
        #优先--config指定与位置参数
        config_path = args.config or args.config_pos or "config.yaml"
    
        #读取配置
        csv_path, col_x, col_y = load_cfg (CONFIG_PATH = "config.yaml")

        #避免当前目录找不到CSV文件
        if not os.path.isabs(csv_path) and not os.path.exists(csv_path):
            alt = os.path.join(os.path.dirname(os.path.abspath(config_path)),csv_path)
            if os.path.exists(alt):
                csv_path = alt
        
        #读取数据
        xs, ys = load_csv (csv_path, col_x, col_y)
    
        #获得一般计算结果
        n, mean_x, mean_y, r = corr_math(xs, ys)
        
        #获得numpy计算结果
        r_np = corr_numpy(xs,ys)
        
        print("n =", n)
        print("mean_x =", mean_x)
        print("mean_y =", mean_y)
        print("r =", r)
        print("r_np =", r_np)

    #使错误统一输出，同时统一退出码
    except CorrToolError as e:
        print("[ERROR]",e,file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print("[ERROR]未知错误:", e,file=sys.stderr)
        sys.exit(1)