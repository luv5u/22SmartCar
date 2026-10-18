from legacy_corr import corr_math, load_csv

def test_positive():
    # [1,2,3] 与 [2,4,6] 应该完全正相关
    n, mean_x, mean_y, r = corr_math([1, 2, 3], [2, 4, 6])
    if abs(r - 1.0) > 1e-9:
        print("失败：正相关算错了，r =", r)
    else:
        print("通过：正相关")

def test_negative():
    # [1,2,3] 与 [6,4,2] 应该完全负相关
    n, mean_x, mean_y, r = corr_math([1, 2, 3], [6, 4, 2])
    if abs(r + 1.0) > 1e-9:
        print("失败：负相关算错了，r =", r)
    else:
        print("通过：负相关")

def test_zero_variance():
    # 第二列恒定，方差为 0，应该报错
    try:
        corr_math([1, 2, 3], [5, 5, 5])
        print("失败：零方差没有报错")
    except ZeroDivisionError:
        print("通过：零方差")


def test_empty_file():
    # 空文件应该报错
    open("empty.csv", "w").close()
    try:
        load_csv("empty.csv", "a", "b")
        print("失败：空文件没有报错")
    except Exception:
        print("通过：空文件")

if __name__ == "__main__":
    test_positive()
    test_negative()
    test_zero_variance()
    test_empty_file()
    print("测试结束")