#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "October 23, 2024"
__project__   = "BRUCE"
__version__   = "0.0.7"
__status__    = "Product"

'''
#在 BRUCE 项目中,使用Numba 进行在线前编译(AOT)以优化运动学计算。
Compile BRUCE kinematics ahead of time (AOT) using Numba
'''

from numba.pycc import CC  #从 numba.pycc 模块中导入 CC 类，用于提前编译 Python 代码
from Settings.BRUCE_macros import *  #导入BRUCE 项目的所有宏定义和设置

#获取右侧髋关节旋转中心的坐标信息
lx = COORDINATE_VECTOR[HIP_YAW_R][0]
ly = COORDINATE_VECTOR[HIP_YAW_R][1]
lz = COORDINATE_VECTOR[HIP_YAW_R][2]

# 获取膝关节和踝关节的相关参数
# d3 表示膝关节的长度，d4 表示踝关节的长度
d3 = COORDINATE_VECTOR[KNEE_PITCH_R][0]
d4 = COORDINATE_VECTOR[ANKLE_PITCH_R][0]

#定义右腿和左腿的位移向量
p_b1_r = np.array([lx, +ly, lz])
p_b1_l = np.array([lx, -ly, lz])

#定义不同位置的变换矩阵
#T_5a表示踝关节的齐次变换矩阵
T_5a = np.eye(4)   #初始化为 4x4 单位矩阵
T_5a[0:3, 0:3] = MF.Ry(-PI_2) @ MF.Rz(PI_2) #应用绕y 轴和z 轴的旋转   
T_5a[0:3,   3] = np.array([0, 0, 0]) #没有平移分量

T_5s = np.eye(4)   #表示脚底的齐次变换矩阵
T_5s[0:3, 0:3] = MF.Ry(-PI_2) @ MF.Rz(PI_2)#应用绕y轴和z轴的旋
T_5s[0:3,   3] = np.array([a5, 0, 0])# 平移a5

#T_5t 表示脚趾的齐次变换矩阵
T_5t = np.eye(4)
T_5t[0:3, 0:3] = MF.Ry(-PI_2) @ MF.Rz(PI_2)
T_5t[0:3,   3] = np.array([a5, at, 0])#平移a5和at

# T_5h 表示脚跟的齐次变换矩阵
T_5h = np.eye(4)
T_5h[0:3, 0:3] = MF.Ry(-PI_2) @ MF.Rz(PI_2)
T_5h[0:3,   3] = np.array([a5, -ah, 0]) #平移a5和-ah


#创建一个用于BRUCE运动学的编译上下文
cc = CC('BRUCE_kinematics')

#导出前向运动学函数leg_FK
@cc.export('leg_FK', '(f8, f8, f8[:])')
def leg_FK(leg, option, q):
    # leg    - right (+1)
    #        -  left (-1)
    # option - ankle (0)
    #        - sole  (1)
    #        - toe   (2)
    #        - heel  (3)
    #计算指定腿的前向运动学
    #leg：指定是右腿（+1）还是左腿（-1)
    #option：指定计算的部分（踝关节、脚底、脚趾或脚跟）
    #q：关节角度数组
    if option == 0:
        T_5f = T_5a #踝关节
    elif option == 1:
        T_5f = T_5s ##脚底
    elif option == 2:
        T_5f = T_5t #脚趾
    elif option == 3:
        T_5f = T_5h #脚跟

    # 计算基座到第一关节的齐次变换矩阵
    T_b1 = np.eye(4)
    T_b1[0:3, 0:3] = MF.Rz(q[0])  #绕 z 轴的旋转，表示髋关节的旋转
    T_b1[0:3,   3] = p_b1_r if leg == +1 else p_b1_l #平移到右腿或左腿基座的
    
    #计算第一关节到第二关节的齐次变换矩阵
    T_12 = np.eye(4)
    T_12[0:3, 0:3] = MF.Rx(+PI_2) @ MF.Rz(q[1] - PI_2)

    T_23 = np.eye(4)
    T_23[0:3, 0:3] = MF.Rx(-PI_2) @ MF.Rz(q[2])

    T_34 = np.eye(4)
    T_34[0:3, 0:3] = MF.Rx(+PI_2) @ MF.Rz(q[3])
    T_34[  0,   3] = d3

    T_45 = np.eye(4)
    T_45[0:3, 0:3] = MF.Rz(q[4])
    T_45[  0,   3] = d4

    T_bf = T_b1 @ T_12 @ T_23 @ T_34 @ T_45 @ T_5f
    R_bf = T_bf[0:3, 0:3]
    p_bf = T_bf[0:3,   3]

    return R_bf, p_bf


@cc.export('leg_IK', '(f8, f8, f8[:], f8[:], f8[:])')
def leg_IK(leg, option, x_bf, p_bf, q0):
    # leg    - right (+1)
    #        -  left (-1)
    # option - ankle only for now
    r11 = x_bf[0]
    r21 = x_bf[1]
    r31 = x_bf[2]
    px  = p_bf[0]
    py  = p_bf[1]
    pz  = p_bf[2]

    rx = px - lx
    ry = py - ly if leg == +1 else py + ly
    rz = pz - lz

    c4 = MF.sat((rx * rx + ry * ry + rz * rz - d3 * d3 - d4 * d4) / 2. / d3 / d4, -1, +1)
    s4 = -np.sqrt(1 - c4 * c4)

    tx = rz * r21 - ry * r31
    ty = rx * r31 - rz * r11
    tz = ry * r11 - rx * r21

    alpha = np.zeros(2)
    beta  = np.zeros(2)
    gamma = np.zeros(2)

    k0 = rx * ty - ry * tx
    if np.abs(k0) > 0.01:
        k1 = (ry * tz - rz * ty) / k0
        k2 = d4 * s4 * ty / k0
        k3 = (rz * tx - rx * tz) / k0
        k4 = -d4 * s4 * tx / k0
        k5 = k1 * k1 + k3 * k3 + 1
        k6 = 2 * (k1 * k2 + k3 * k4)
        k7 = k2 * k2 + k4 * k4 - 1

        delta    = k6 * k6 - 4 * k5 * k7
        delta_sr = np.sqrt(delta) if delta > 0 else 0
        gamma[0] = (+delta_sr - k6) / 2 / k5
        gamma[1] = (-delta_sr - k6) / 2 / k5
        for i in range(2):
            alpha[i] = k1 * gamma[i] + k2
            beta[i]  = k3 * gamma[i] + k4
    else:
        k0 = ry * tz - rz * ty
        if np.abs(k0) > 0.01:
            k1 = (rz * tx - rx * tz) / k0
            k2 = d4 * s4 * tz / k0
            k3 = (rx * ty - ry * tx) / k0
            k4 = -d4 * s4 * ty / k0
            k5 = k1 * k1 + k3 * k3 + 1
            k6 = 2 * (k1 * k2 + k3 * k4)
            k7 = k2 * k2 + k4 * k4 - 1

            delta    = k6 * k6 - 4 * k5 * k7
            delta_sr = np.sqrt(delta) if delta > 0 else 0
            alpha[0] = (+delta_sr - k6) / 2 / k5
            alpha[1] = (-delta_sr - k6) / 2 / k5
            for i in range(2):
                beta[i]  = k1 * alpha[i] + k2
                gamma[i] = k3 * alpha[i] + k4
        else:
            k0 = rx * tz - rz * tx
            k1 = (rz * ty - ry * tz) / k0
            k2 = d4 * s4 * tz / k0
            k3 = (ry * tx - rx * ty) / k0
            k4 = -d4 * s4 * tx / k0
            k5 = k1 * k1 + k3 * k3 + 1
            k6 = 2 * (k1 * k2 + k3 * k4)
            k7 = k2 * k2 + k4 * k4 - 1

            delta    = k6 * k6 - 4 * k5 * k7
            delta_sr = np.sqrt(delta) if delta > 0 else 0
            beta[0] = (+delta_sr - k6) / 2 / k5
            beta[1] = (-delta_sr - k6) / 2 / k5
            for i in range(2):
                alpha[i] = k1 * beta[i] + k2
                gamma[i] = k3 * beta[i] + k4
    
    num_sol = 2
    q = np.zeros((num_sol, 5))
    for i in range(num_sol):
        # t4
        q[i, 3] = np.arctan2(s4, c4)

        # t2
        s2 = MF.sat(gamma[i], -1, +1)
        c2 = np.sqrt(1 - s2 * s2)
        q[i, 1] = np.arctan2(s2, c2)

        # t1
        if c2 > 0.01:
            q[i, 0] = np.arctan2(beta[i], alpha[i])
        else:  # alpha, beta = 0
            q[i, 0] = q0[0]
        s1 = np.sin(q[i, 0])
        c1 = np.cos(q[i, 0])

        # t3
        s3 = (ry * c1 - rx * s1) / (d3 + d4 * c4)
        c3 = ((rx * c1 + ry * s1) * s2 - rz * c2) / (d3 + d4 * c4)
        q[i, 2] = np.arctan2(s3, c3)

        # t5
        s45 = (r31 * c2 - (r11 * c1 + r21 * s1) * s2) / c3  if np.abs(c3) > 0.001 else (r11 * s1 - r21 * c1) / s3
        c45 = (r11 * c1 + r21 * s1) * c2 + r31 * s2
        q[i, 4] = np.arctan2(s45, c45) - q[i, 3]

    return q


@cc.export('leg_Jacobian', '(f8, f8, f8[:], f8[:])')
def leg_Jacobian(leg, option, q, dq):
    # leg    - right (+1)
    #        -  left (-1)
    # option - ankle (0)
    #        - sole  (1)
    #        - toe   (2)
    #        - heel  (3)
    if option == 0:
        T_5f = T_5a
    elif option == 1:
        T_5f = T_5s
    elif option == 2:
        T_5f = T_5t
    elif option == 3:
        T_5f = T_5h

    Jw  = np.zeros((3, 5))
    Jv  = np.zeros((3, 5))
    dJw = np.zeros((3, 5))
    dJv = np.zeros((3, 5))
    w   = np.zeros((3, 5))
    p   = np.zeros((3, 6))
    T   = np.zeros((5, 4, 4))

    T[0,   3,   3] = 1
    T[0, 0:3, 0:3] = MF.Rz(q[0])
    T[0, 0:3,   3] = p_b1_r if leg == +1 else p_b1_l

    T[1,   3,   3] = 1
    T[1, 0:3, 0:3] = MF.Rx(+PI_2) @ MF.Rz(q[1] - PI_2)

    T[2,   3,   3] = 1
    T[2, 0:3, 0:3] = MF.Rx(-PI_2) @ MF.Rz(q[2])

    T[3,   3,   3] = 1
    T[3, 0:3, 0:3] = MF.Rx(+PI_2) @ MF.Rz(q[3])
    T[3,   0,   3] = d3

    T[4,   3,   3] = 1
    T[4, 0:3, 0:3] = MF.Rz(q[4])
    T[4,   0,   3] = d4

    T_bi = np.eye(4)
    w_bi = np.zeros(3)
    for i in range(5):
        T_bi  = T_bi @ np.copy(T[i, :, :])
        z_bi  = T_bi[0:3, 2]
        w_bi += z_bi * dq[i]

        w[:, i]   = w_bi
        p[:, i]   = T_bi[0:3, 3]
        Jw[:, i]  = z_bi
        dJw[:, i] = MF.hat(w_bi) @ np.copy(z_bi)
    
    T_bf = T_bi @ T_5f
    p[:, 5] = T_bf[0:3, 3]
    for i in range(5):
        dp = p[:, 5] - p[:, i]
        Jv[:, i]  = MF.hat( Jw[:, i]) @ dp
        dJv[:, i] = MF.hat(dJw[:, i]) @ dp

    ddp = np.zeros(3)
    for i in range(4, -1, -1):
        ddp += MF.hat(w[:, i]) @ (p[:, i+1] - p[:, i])
        dJv[:, i] += MF.hat(Jw[:, i]) @ ddp

    return T_bf[0:3, 0:3], w_bi, T_bf[0:3, 3], Jv @ np.copy(dq), Jw, dJw @ np.copy(dq), Jv, dJv @ np.copy(dq)


@cc.export('base2task', '(f8,'
                        ' f8[:, :], f8[:], f8[:],    f8[:],'
                        ' f8[:, :], f8[:], f8[:],    f8[:],'
                        ' f8[:, :], f8[:], f8[:, :], f8[:])')
def base2task(leg,
              R_wb, w_bb, p_wb, v_bb, 
              R_bf, w_bf, p_bf, v_bf, 
              Jw_bf, dJwdq_bf, Jv_bf, dJvdq_bf):
    R_wb = np.copy(R_wb)
    R_bf = np.copy(R_bf)
    w_bf = np.copy(w_bf)
    p_bf = np.copy(p_bf)

    R_wf = R_wb @ R_bf
    R_fb = R_bf.T
    w_ff = R_fb @ w_bf

    p_wf = p_wb + R_wb @ p_bf
    v_wf = R_wb @ (v_bb + MF.hat(w_bb) @ p_bf + v_bf)

    if leg == +1:
        Jw_ff = R_fb @ np.hstack((    np.eye(3), np.zeros((3, 3)), Jw_bf, np.zeros((3, 5))))
        Jv_wf = R_wb @ np.hstack((-MF.hat(p_bf),        np.eye(3), Jv_bf, np.zeros((3, 5))))
    elif leg == -1:
        Jw_ff = R_fb @ np.hstack((    np.eye(3), np.zeros((3, 3)), np.zeros((3, 5)), Jw_bf))
        Jv_wf = R_wb @ np.hstack((-MF.hat(p_bf),        np.eye(3), np.zeros((3, 5)), Jv_bf))
    
    dJwdq_ff = R_fb @ (dJwdq_bf + MF.hat(w_bb) @ w_bf)
    dJvdq_wf = R_wb @ (dJvdq_bf + MF.hat(w_bb) @ (v_bb + MF.hat(w_bb) @ p_bf + 2 * v_bf))

    return R_wf, w_ff, p_wf, v_wf, Jw_ff, dJwdq_ff, Jv_wf, dJvdq_wf


# bear2joint
T_bj_r = np.array([[-1,    0,    0,  0,  0],
                   [ 0, +0.5, -0.5,  0,  0],
                   [ 0, -0.5, -0.5,  0,  0],
                   [ 0,    0,    0, -1,  0],
                   [ 0,    0,    0, +1, +1]])

T_bj_l = np.array([[-1,    0,    0,  0,  0],
                   [ 0, +0.5, -0.5,  0,  0],
                   [ 0, -0.5, -0.5,  0,  0],
                   [ 0,    0,    0, +1,  0],
                   [ 0,    0,    0, -1, -1]])

# joint2bear
T_jb_r = np.linalg.inv(T_bj_r)
T_jb_l = np.linalg.inv(T_bj_l)

# iq2torque
T_it_r = np.diag([IQ2TORQUE] * 5)
T_it_l = np.diag([IQ2TORQUE] * 5)

# torque2iq
T_ti_r = np.diag([TORQUE2IQ] * 5)
T_ti_l = np.diag([TORQUE2IQ] * 5)


@cc.export('joint2bear', '(f8, f8, f8[:])')
def joint2bear(leg, option, q):
    # leg    -    right (+1)
    #        -     left (-1)
    # option - position (0)
    #        - velocity (1)
    #        -   torque (2)
    q = np.copy(q)
    if leg == +1:
        if option == 0:
            b = T_jb_r @ q
        elif option == 1:
            b = T_jb_r @ q
        elif option == 2:
            b = T_ti_r @ T_bj_r.T @ q
    elif leg == -1:
        if option == 0:
            b = T_jb_l @ q
        elif option == 1:
            b = T_jb_l @ q
        elif option == 2:
            b = T_ti_l @ T_bj_l.T @ q
    return b


@cc.export('bear2joint', '(f8, f8, f8[:])')
def bear2joint(leg, option, b):
    # leg    -    right (+1)
    #        -     left (-1)
    # option - position (0)
    #        - velocity (1)
    #        -   torque (2)
    b = np.copy(b)
    if leg == +1:
        if option == 0:
            q = T_bj_r @ b
        elif option == 1:
            q = T_bj_r @ b
        elif option == 2:
            q = T_jb_r.T @ T_it_r @ b
    elif leg == -1:
        if option == 0:
            q = T_bj_l @ b
        elif option == 1:
            q = T_bj_l @ b
        elif option == 2:
            q = T_jb_l.T @ T_it_l @ b
    return q


if __name__ == '__main__':
    cc.compile()