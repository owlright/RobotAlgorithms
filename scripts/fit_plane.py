import numpy as np
import matplotlib.pyplot as plt
# # 获取matplotlib的字体列表
# import matplotlib
# a=sorted([f.name for f in matplotlib.font_manager.fontManager.ttflist])
# for (i, font_name) in enumerate(a):
#     print(f"{i}: {font_name}")
plt.rcParams['font.family'] = 'LXGW WenKai GB Screen'

def plot_3d_points(matrix, title="3D点可视化"):
    """
    绘制3D点的散点图

    参数:
    matrix: numpy数组，每行表示一个3D点 (x, y, z)
    title: 图形标题
    """
    # 创建3D图形
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # 提取x, y, z坐标
    x = matrix[:, 0]
    y = matrix[:, 1]
    z = matrix[:, 2]

    # 绘制散点图
    scatter = ax.scatter(x, y, z, c=range(len(matrix)), cmap='viridis', s=10)

    # 添加点的标签
    for i, (xi, yi, zi) in enumerate(zip(x, y, z)):
        ax.text(xi, yi, zi, f'  P{i}({xi},{yi},{zi})', fontsize=8)

    # 设置坐标轴标签
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(title)

    # 添加颜色条
    plt.colorbar(scatter)

    # 显示图形
    plt.show()

def plot_3d_points_with_plane(matrix, plane_params, title="3D点与拟合平面"):
    """
    绘制3D点和拟合平面

    参数:
    matrix: numpy数组，每行表示一个3D点 (x, y, z)
    plane_params: 拟合平面的参数 (a, b, c, d) 对应于方程 ax + by + cz + d = 0
    title: 图形标题
    """
    # 创建3D图形
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # 提取x, y, z坐标
    x = matrix[:, 0]
    y = matrix[:, 1]
    z = matrix[:, 2]

    # 绘制散点图
    scatter = ax.scatter(x, y, z, c=range(len(matrix)), cmap='viridis', s=10)

    # 添加点的标签
    for i, (xi, yi, zi) in enumerate(zip(x, y, z)):
        ax.text(xi, yi, zi, f'  P{i}({xi},{yi},{zi})', fontsize=8)

    # 设置坐标轴标签
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(title)

    # 添加颜色条
    plt.colorbar(scatter)

    # 绘制拟合平面
    a, b, c, d = plane_params
    xx, yy = np.meshgrid(np.linspace(min(x), max(x), 10), np.linspace(min(y), max(y), 10))
    zz = (-d - a * xx - b * yy) / c
    ax.plot_surface(xx, yy, zz, alpha=0.5)

    # 显示图形
    plt.show()
A = np.array([
    [1.0, 2.0, 3.0],
    [4.0, 5.0, 6.0],
    [7.0, 8.0, 9.0],
    [2.0, 3.0, 1.0],
    [3.0, 4.0, 2.0],
    [5.0, 6.0, 4.0],
    [6.0, 7.0, 5.0],
    [8.0, 9.0, 7.0]
])
plot_3d_points(A, title="3D点可视化")

# 进行SVD分解
# 添加最后一列都为1，因为a*x+b*y+c*z+d=0 可以看成是(x, y, z, 1) * [a, b, c, d] = 0
A_ext = np.column_stack([A, np.ones(A.shape[0])])
U, s, Vt = np.linalg.svd(A_ext)

print("U矩阵形状:", U.shape)
print("奇异值s:", s)
print("Vt矩阵形状:", Vt.shape)

# # 验证分解结果
# S = np.zeros_like(A, dtype=float)
# S[:min(A.shape), :min(A.shape)] = np.diag(s)
# A_reconstructed = U @ S @ Vt
# print(A)

# 找到拟合平面的法向量
a, b, c, d = Vt[-1, :]  # 最后一个奇异向量对应于最小奇异值
print(f"拟合平面的方程: {a}x + {b}y + {c}z + {d} = 0")
plot_3d_points_with_plane(A, (a, b, c, d), title="3D点与拟合平面")