import yaml
import csv
import math
import numpy as np

#封装yaml读取与配置读取
def load_cfg (CONFIG_PATH = "config.yaml"): #存储yaml文件路径

    with open(CONFIG_PATH) as f:
        cfg = yaml.safe_load(f)

    #将其中配置提取出
    csv_path = cfg["input_csv"]
    col_x = cfg["columns"]["x"]
    col_y = cfg["columns"]["y"]

    return csv_path, col_x, col_y

#封装csv内数据读取
def load_csv (csv_path, col_x, col_y):
    xs = []
    ys = []

    with open(csv_path) as f:
        reader = csv.DictReader(f)
        #将读取的数据转化为列表
        for row in reader:
            xs.append(float(row[col_x]))
            ys.append(float(row[col_y]))
    return xs, ys

#封装一般方式计算相关系数
def corr_math (xs, ys):
    n = len(xs)
    
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


if __name__ == '__main__': #保护
    #读取配置
    csv_path, col_x, col_y = load_cfg (CONFIG_PATH = "config.yaml")
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