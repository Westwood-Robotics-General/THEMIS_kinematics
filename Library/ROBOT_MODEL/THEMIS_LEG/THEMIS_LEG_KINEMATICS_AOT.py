#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "October 23, 2024"
__project__   = "BRUCE"
__version__   = "0.0.7"
__status__    = "Product"

'''
Compile BRUCE kinematics ahead of time (AOT) using Numba
'''

from numba.pycc import CC
import numpy as np
from math_function import Ry, Rz, Rx  # 从 math_function.py 导入旋转函数


# 定义常量
lx = 0
ly = 0
lz = 0

d1 = 20.0
d2_leg = 20.0
d3 = 50.0
d4 = 50.0

p_b1_r = np.array([lx, +ly, lz])
p_b1_l = np.array([lx, -ly, lz])

# 定义转换矩阵 T_5a, T_5s, T_5t, T_5h
T_5a = np.eye(4)
T_5a[0:3, 0:3] = Ry(-np.pi / 2) @ Rz(np.pi / 2)  # 使用导入的 Ry 和 Rz 函数
T_5a[0:3, 3] = np.array([0, 0, 0])



# Numba CC 编译
cc = CC('THEMIS_kinematics')

@cc.export('leg_FK', '(f8, f8, f8[:])')
def leg_FK(leg, q):
    


    T_b1 = np.eye(4)
    T_b1[0:3, 0:3] = Ry(+np.pi / 4) @ Rz(q[0])
    T_b1[0:3, 3] = p_b1_r if leg == +1 else p_b1_l

    T_12 = np.eye(4)
    T_12[0:3, 0:3] = Ry(-np.pi / 2) @ Rz(q[1]-np.pi / 2)
    T_12[0, 3] = d1

    T_23 = np.eye(4)
    T_23[0:3, 0:3] = Rx(-np.pi / 4) @ Ry(np.pi / 2) @ Rz(q[2])
    T_23[0, 3] = d2_leg

    T_34 = np.eye(4)
    T_34[0:3, 0:3] = Rz(q[3])
    T_34[0, 3] = d3

    T_45 = np.eye(4)  # 初始化单位矩阵
    T_45[0, 3] = d4

    

    T_bf = T_b1 @ T_12 @ T_23 @ T_34 @ T_45 
    R_bf = T_bf[0:3, 0:3]
    p_bf = T_bf[0:3, 3]

    return R_bf, p_bf    #由于np.pi近似值


@cc.export('leg_IK', '(f8, f8, f8[:], f8[:], f8[:])')
def leg_IK(x_bf, p_bf, q0):



    # 计算每个公式
    sqrt_2 = np.sqrt(2)

    # r13 计算
    r13 = (sqrt_2 * x_bf[0] / 2) - (sqrt_2 * x_bf[2] / 2)

    # px 计算
    px = (-sqrt_2 * lx / 2) + (sqrt_2 * lz / 2) + (sqrt_2 * p_bf[0] / 2) - (sqrt_2 * p_bf[2] / 2)

    # r23 不变
    r23 = x_bf[1]

    # py 计算
    py = -ly + p_bf[1]

    # r33 计算
    r33 = (sqrt_2 * x_bf[0] / 2) + (sqrt_2 * x_bf[2] / 2)

    # pz 计算
    pz = (-sqrt_2 * lx / 2) - (sqrt_2 * lz / 2) + (sqrt_2 * p_bf[0] / 2) + (sqrt_2 * p_bf[2] / 2)
    # 提取输入参数
    # r13 = x_bf[0]
    # r23 = x_bf[1]
    # r33 = x_bf[2]
    # px = p_bf[0]
    # py = p_bf[1]
    # pz = p_bf[2]

    # 第一步：计算 theta2
    theta2_solutions = [
        np.arctan2(r33, np.sqrt(1 - r33**2)),
        np.arctan2(r33, -np.sqrt(1 - r33**2))
    ]
    num_sol = 2
    q = np.zeros((num_sol, 4))

    # 第二步：根据 theta2 求解 theta1 和中间变量 A 和 B
    for i, theta2 in enumerate(theta2_solutions):
        c2 = np.cos(theta2)
        s2 = np.sin(theta2) 
        # print(f"Theta2 solution:\n theta2 = {theta2}")

        # 判断 c2 是否接近零
        if np.isclose(c2, 0):
            theta1 = q0  # 如果 cos(theta2) = 0，则 theta1 赋初值
        else:
            theta1 = np.arctan2(r13 / c2, -r23 / c2)  # 正常计算 theta1

        # 计算 A 和 B
        s1 = np.sin(theta1)
        c1 = np.cos(theta1)
        B = (d2_leg * s2 - pz) / c2
        A = (px + s1 * (-c2 * d2_leg - s2 * B)) / c1 - d1

        # 第三步：计算 k8 和 k9
        k8 = A
        k9 = B

        # 第四步：计算 c4
        c4 = (k8**2 + k9**2 - d3**2 - d4**2) / (2 * d3 * d4)
        # print(f"c4 solution:\n c4 = {c4}")
        # 检查 c4 是否在合法范围内
        if np.abs(c4) <= 1:
            # 计算 theta4
            theta4 = np.arctan2(-np.sqrt(1 - c4**2), c4)

            # 计算 theta3
            s4 = np.sin(theta4)
            theta3 = np.arctan2(
                (d3 + d4 * c4) * k8 - d4 * s4 * k9,
                (d3 + d4 * c4) * k9 + d4 * s4 * k8
            ) - np.pi / 4

            # 存储解
            q[i, :] = [theta1, theta2, theta3, theta4]

    return q



