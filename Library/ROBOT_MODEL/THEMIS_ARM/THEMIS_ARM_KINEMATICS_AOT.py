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
lx = 5
d1 = 10.0
d2 = 20.0



# Numba CC 编译
cc = CC('THEMIS_kinematics')

@cc.export('arm_FK', '(f8, f8, f8[:])')
def arm_FK( q):
    T_01 = np.eye(4)
    T_01[0:3, 0:3] = Ry(np.pi / 2) @ Rz(q[0])
    T_01[0:3,   3] = np.array([0, 0, 0])

    T_12 = np.eye(4)
    T_12[0:3, 0:3] = Rx(np.pi / 2) @ Rz(q[1])
    T_12[0:3,   3]= np.array([0, 0, 0])

    T_23 = np.eye(4)
    T_23[0:3, 0:3] = Rx(-np.pi / 2) @ Rz(q[2])
    T_23[0:3,   3]= np.array([0, 0, 0])

    T_34 = np.eye(4)
    T_34[0:3, 0:3] =  Rx(np.pi / 2)@ Rz(q[3])
    T_34[0:3,   3]= np.array([lx, 0, d1])

    T_45 = np.eye(4)  # 初始化单位矩阵
    T_45[0:3,   3] = np.array([-lx, d2, 0])

    T_bf = T_01 @T_12 @ T_23 @ T_34 @ T_45 
    R_bf = T_bf[0:3, 0:3]
    p_bf = T_bf[0:3,   3]

    return R_bf, p_bf

@cc.export('arm_IK', '(f8, f8, f8[:], f8[:], f8[:])')
def arm_IK(x_bf, p_bf, q0):
    # 提取输入参数
    r13 = x_bf[0]
    px = p_bf[0]
    py = p_bf[1]
    pz = p_bf[2]


    # Step 1: 计算 theta4 的两个可能解
    a = -2 * lx**2 + 2 * d1 * d2
    b = -2 * d1 * lx - 2 * d2 * lx
    c = px**2 + py**2 + pz**2 - d1**2 - d2**2 - 2 * lx**2

    discriminant = a**2 + b**2 - c**2
    if discriminant < 0:
        print("No solution for theta4")
        return np.zeros((2, 4))  # 返回空解矩阵

    sqrt_discriminant = np.sqrt(discriminant)
    theta4_solutions = [
        np.arctan2(sqrt_discriminant, c) + np.arctan2(b, a),
        np.arctan2(-sqrt_discriminant, c) + np.arctan2(b, a)
    ]

    # 初始化解数组，假设最多有 2 个 theta4，每个 theta4 可能对应 2 个 theta2
    num_sol = 4  # 最大可能解数量（2 个 theta4 * 每个 theta4 对应 2 个 theta2）
    q = np.zeros((num_sol, 4))  # 每行存储一组解：[theta1, theta2, theta3, theta4]

    # 当前找到的解的数量
    solution_count = 0

    # Step 2: 计算每个 theta4 解对应的 theta2 解集
    def solve_theta2_and_AB(theta4):
        c4 = np.cos(theta4)
        s4 = np.sin(theta4)
        A = d2 * c4 - lx * s4 + d1
        B = lx - lx * c4 - d2 * s4

        theta2_solutions = []
        
        # B 为 0 的特殊情况
        if B == 0:
            b = px / A
            if -1 <= b <= 1:
                theta2_solutions = [
                    np.arctan2(np.sqrt(1 - b**2), b),
                    np.arctan2(-np.sqrt(1 - b**2), b)
                ]
            return theta2_solutions, A, B

        # 常规情况，计算 k1、k2 和 k3
        k1 = (A**2 / B**2) + 1
        k2 = (-2 * px * A) / B**2
        k3 = (px**2 / B**2) + r13**2 - 1
     
        discriminant = k2**2 - 4 * k1 * k3
        if discriminant < 0:
            return theta2_solutions, A, B

        sqrt_discriminant = np.sqrt(discriminant)
        gamma_solutions = [
            (-k2 + sqrt_discriminant) / (2 * k1),
            (-k2 - sqrt_discriminant) / (2 * k1)
        ]

        for gamma in gamma_solutions:
            if -1 <= gamma <= 1:
                theta2_solutions.append(np.arctan2(np.sqrt(1 - gamma**2), gamma))
                theta2_solutions.append(np.arctan2(-np.sqrt(1 - gamma**2), gamma))

        return theta2_solutions, A, B

    def solve_theta1(s3, c3, c2, s2, A, B, py, pz):
        a = s3 * B
        c = c2 * c3 * B - s2 * A
        d = py
        e = s2 * A - c2 * c3 * B
        f = s3 * B
        g = pz

        # 确保矩阵的行列式非零
        determinant = a * f - c * e
        if determinant == 0:
            print("No solution for theta1 (determinant is zero)")
            return None

        # 计算 sin(theta1) 和 cos(theta1)
        sin_theta1 = (a * g - d * e) / determinant
        cos_theta1 = (d * f - c * g) / determinant

        # 检查 sin 和 cos 是否在有效范围内
        if not (-1 <= sin_theta1 <= 1) or not (-1 <= cos_theta1 <= 1):
            print("No valid solution for theta1 within the range")
            return None

        # 使用 atan2 计算 theta1
        theta1 = np.arctan2(sin_theta1, cos_theta1)
        return theta1

    for theta4 in theta4_solutions:
        theta2_solutions, A, B = solve_theta2_and_AB(theta4)

        # Step 3: 对每个 theta2 解，计算对应的 theta3 和 theta1
        for theta2 in theta2_solutions:
            s2 = np.sin(theta2)
            c2 = np.cos(theta2)
            s4 = np.sin(theta4)
            c4 = np.cos(theta4)

            # 计算 theta3
            if s2 != 0:
                theta3 = np.arctan2(r13 / s2, (px - c2 * A) / (s2 * B))
            else:
                theta3 = 0

            # 计算 theta1 使用 solve_theta1 函数
            s3 = np.sin(theta3)
            c3 = np.cos(theta3)
            theta1 = solve_theta1(s3, c3, c2, s2, A, B, py, pz)
            if theta1 is None:
                continue  # 如果 theta1 无解，跳过该解

            # Step 4: 存储解
            if solution_count < num_sol:  # 确保不超出预定的解数组大小
                q[solution_count, 0] = theta1
                q[solution_count, 1] = theta2
                q[solution_count, 2] = theta3
                q[solution_count, 3] = theta4
                solution_count += 1

    # 返回包含所有解的完整 q 数组
    return q