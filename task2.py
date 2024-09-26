import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import roboticstoolbox as rbt
from spatialmath.base import rotx, trotx,troty,trotz

# 创建图形对象
fig = plt.figure()

def setup_figure(axs):
    for i,ax in enumerate(axs):
        ax.set_xlim([-12, 12])
        ax.set_ylim([-12, 12])
        ax.set_zlim([0, 20])
        # 设置标签和标题
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        if i == 0:
            ax.set_title('3D Circular Platforms')
        else:
            ax.set_title(f"pose {i}")


def calculate_transmatrix(variable):
    a = trotx(variable[3], "deg")
    b = troty(variable[4], "deg")
    c = trotz(variable[5], "deg")
    trans = np.array([
        [1, 0, 0, variable[0]],
        [0, 1, 0, variable[1]],
        [0, 0, 1, variable[2]],
        [0, 0, 0, 1]
    ])
    transmatrix = a @ b @ c @ trans
    return transmatrix

ax0 = fig.add_subplot(241, projection='3d')
ax1 = fig.add_subplot(242, projection='3d')
ax2 = fig.add_subplot(243, projection='3d')
ax3 = fig.add_subplot(244, projection='3d')
ax4 = fig.add_subplot(245, projection='3d')
ax5 = fig.add_subplot(246, projection='3d')
ax6 = fig.add_subplot(247, projection='3d')
axs = [ax0,ax1,ax2,ax3,ax4,ax5,ax6]
setup_figure(axs)
# 设置坐标轴范围


# 绘制底部半径为9的圆形平台
theta = np.linspace(0, 2 * np.pi, 100)
circlex1 = 9 * np.cos(theta)
circley1 = 9 * np.sin(theta)
circlez1 = np.zeros_like(theta)  # 底部平台在 z = 0 处
for ax in axs:
    ax.plot(circlex1, circley1, circlez1, color='blue', alpha=0.8)


# 绘制上方15单位处的直径为5的圆形平台
circlex2 = 5 * np.cos(theta)
circley2 = 5 * np.sin(theta)
circlez2 = np.ones_like(theta) * 15  # 上方平台在 z = 15 处
ax0.plot(circlex2, circley2, circlez2, color='red', alpha=0.8)
circle = np.row_stack((circlex2, circley2, circlez2-15, np.ones_like(circlex2)))

# 底部圆形平台参数
r1 = 9
theta1_degrees = [-15, 75, 105, 195, 225, -45]
theta1 = np.radians(theta1_degrees)  # 将角度转换为弧度
x1 = r1 * np.cos(theta1)
y1 = r1 * np.sin(theta1)
z1 = np.zeros_like(theta1)  # z = 0

# 上方圆形平台参数
r2 = 5
theta2_degrees = [0, 60, 120, 180, 240, 320]
theta2 = np.radians(theta2_degrees)  # 将角度转换为弧度
x2 = r2 * np.cos(theta2)
y2 = r2 * np.sin(theta2)
z2 = np.ones_like(theta2) * 15  # z = 15

# 在大圆上绘制标志点
for ax in axs:
    ax.scatter(x1, y1, z1, color='blue', s=30, label='Large Circle Points')

# 在小圆上绘制标志点
ax0.scatter(x2, y2, z2, color='red', s=20, label='Small Circle Points')

# 绘制两个圆上点之间的连接线
for i in range(len(x1)):
    ax0.plot([x1[i], x2[i]], [y1[i], y2[i]], [z1[i], z2[i]], color='green')

variables = np.array([[0,0,15,0,0,0],
                    [-5,0,15,0,0,0],
                    [5,5,15,0,0,0],
                    [5,0,15,15,0,0],
                    [5,2,15,10,10,0],
                     [0,0,15,10,10,10]])

for i,variable in enumerate(variables):
    transmatrix = calculate_transmatrix(variable)
    # 生成齐次坐标 [x, y, z, 1]
    Ps_homogeneous_coords = np.row_stack((x2, y2, z2-15, np.ones_like(x2)))

    # 生成齐次坐标 [x, y, z, 1]
    Bs_homogeneous_coords = np.row_stack((x1, y1, z1, np.ones_like(x1)))
    q = transmatrix@Ps_homogeneous_coords

    L = q - Bs_homogeneous_coords
    for id,A in enumerate(L.T):
        print(f"the vector of L{id} is {A} and the lenth of link{id} = {np.linalg.norm(A)}")

    axs[i+1].scatter(q[0], q[1], q[2], color='green', s=20, label='Large Circle Points')
    for n in range(len(x1)):
        axs[i+1].plot([x1[n], q[0][n]], [y1[n], q[1][n]], [z1[n], q[2][n]], color='red')
    transcircle = transmatrix @ circle
    axs[i + 1].plot(transcircle[0],transcircle[1],transcircle[2], color='blue', alpha=0.8)
# 显示图形
plt.show()

